/*
 * leg_node.cpp
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#include "system/foundation/cfg_reader.h"
#include <boost/algorithm/string.hpp>
#include <repository/resource/force_sensor.h>
#include <repository/resource/joint.h>
#include <system/foundation/utf.h>
#include <system/platform/sw_node/leg_node.h>
#include "system/platform/protocol/qr_protocol.h"

#include <iomanip>

namespace middleware {

struct __PrivateLinearParams {
  double base;
  double offset;
  double k_cmd;
  double b_cmd;
};

LegNode::LegNode(const MiiString& __l)
  : SWNode(__l), leg_(LegType::UNKNOWN_LEG),
    td_(nullptr) {
  for (auto& c : jnt_cmds_)
    c = nullptr;

  for (auto& c : jnt_mods_)
    c = nullptr;
}

LegNode::~LegNode() {
  for (auto& jnt : jnts_by_type_) {
    jnt = nullptr;
  }

  td_ = nullptr;

  for (auto& p : jnt_params_) {
    delete p;
    p = nullptr;
  }
}

bool LegNode::init() {
  if (!SWNode::init())     return false;
  auto cfg = MiiCfgReader::instance();

  MiiString tmp_str;
  cfg->get_value_fatal(getLabel(), "leg", tmp_str);
  boost::to_lower(tmp_str);
  if (0 == tmp_str.compare("fl")) {
    leg_ = LegType::FL;
  } else if (0 == tmp_str.compare("fr")) {
    leg_ = LegType::FR;
  } else if (0 == tmp_str.compare("hl")) {
    leg_ = LegType::HL;
  } else if (0 == tmp_str.compare("hr")) {
    leg_ = LegType::HR;
  } else {
    LOG_ERROR << "Error the 'leg' TAG(" << tmp_str << ") in the 'joint' TAG, "
        << "require 'fl', 'fr', 'hl' or 'hr'";
    return false;
  }

  int count = 0;
  jnts_by_type_.resize(JntType::N_JNTS);
  jnt_params_.resize(JntType::N_JNTS);
  MiiString tag = Label::make_label(getLabel(), "joint_0");
  while(cfg->get_value(tag, "label", tmp_str)) {
    Joint* jnt = Label::getHardwareByName<Joint>(tmp_str);
    LOG_DEBUG << getLabel() << "'s joint_" << count
        << ": " << tmp_str << ",\t" << jnt;
    if (nullptr == jnt) {
      LOG_WARNING << "Can't get joint '" << tmp_str
          << "' pointer from LabelSystem.";
      tag = Label::make_label(getLabel(), "joint_" + std::to_string(++count));
      continue;
    }
    jnts_by_type_[jnt->joint_type()] = jnt;
    auto param  = new __PrivateLinearParams;
    cfg->get_value_fatal(tag, "base", param->base);
    cfg->get_value_fatal(tag, "offset", param->offset);
    cfg->get_value_fatal(tag, "k_cmd", param->k_cmd);
    cfg->get_value_fatal(tag, "b_cmd", param->b_cmd);
    jnt_params_[jnt->joint_type()]  = param;
    jnt_cmds_[jnt->joint_type()]       = jnt->joint_command_const_pointer();
    jnt_mods_[jnt->joint_type()]       = jnt->joint_command_mode_const_pointer();

    tag = Label::make_label(getLabel(), "joint_" + std::to_string(++count));
  }

  tag = Label::make_label(getLabel(), "touchdown");
  if ((cfg->get_value(tag, "label", tmp_str))
      && (td_ = Label::getHardwareByName<ForceSensor>(tmp_str))) {
    LOG_DEBUG << getLabel() << "'s TD: " << td_->getLabel() << "\t" << td_;
    return true;
  } else {
    LOG_WARNING << "The touchdown parameter is not found.";
    return false;
  }
}

void LegNode::updateFromBuf(const unsigned char* __p) {
  /*printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
      __p[0], __p[1], __p[2], __p[3], __p[4], __p[5], __p[6], __p[7]);*/
  int offset  = 0;
  short count = 0;
  double pos  = 0.0;
  for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
    memcpy(&count, __p + offset, sizeof(count));

    pos = 3600 * (double)count / 4096 * 10;
    pos = jnt_params_[type]->base * (pos - jnt_params_[type]->offset);
    pos = pos * 314.15926 / 180;
    pos = pos / 10000;
    
    jnts_by_type_[type]->updateJointPosition(pos);
    offset += sizeof(count); // each count will stand two bytes.
  }

  td_->updateForceCount((__p[offset] | (__p[offset + 1] << 8)));
}

void LegNode::handleMsg(const Packet& pkt) {
  if (pkt.node_id != node_id_) {
    LOG_ERROR << "Wrong match id between Packet and Joint";
    return;
  }

  switch (pkt.msg_id) {
  case MII_MSG_HEARTBEAT_MSG_1:
    // parse the joint state and touchdown data
    updateFromBuf(pkt.data);
    break;
  default:
    SWNode::handleMsg(pkt);
  }
}

bool LegNode::generateCmd(std::vector<Packet>& pkts) {
  int offset  = 0;
  short count = 0;
  bool is_any_valid = false;
  Packet cmd{node_id_, MII_MSG_COMMON_DATA_1, 6, {0}};

  for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
    if (jnts_by_type_[type]->new_command_) {
      is_any_valid = true;
      count = (short)(*jnt_cmds_[type] * jnt_params_[type]->k_cmd + jnt_params_[type]->b_cmd);
      memcpy(cmd.data + offset, &count, sizeof(count));
      jnts_by_type_[type]->new_command_ = false;
    } else {
      cmd.data[offset]     = INVALID_BYTE;
      cmd.data[offset + 1] = INVALID_BYTE;
    }
    offset += 2; // Each count stand two bytes.
  }

  if (is_any_valid) pkts.push_back(cmd);
  return is_any_valid;
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::LegNode, middleware::Label)
