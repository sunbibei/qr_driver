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

LegNode::LegNode(const MiiString& __l)
  : SWNode(__l), leg_(LegType::UNKNOWN_LEG), td_(nullptr) {
  for (auto& c : jnt_cmds_)
    c = nullptr;

  for (auto& c : jnt_mods_)
    c = nullptr;
}

LegNode::~LegNode() {
  for (auto& jnt : joints_by_type_) {
    jnt = nullptr;
  }

  td_ = nullptr;
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
  joints_by_type_.resize(JntType::N_JNTS);
  MiiString tag = Label::make_label(getLabel(), "joint_0");
  while(cfg->get_value(tag, "label", tmp_str)) {
    tag = Label::make_label(getLabel(), "joint_" + std::to_string(++count));
    Joint* jnt = Label::getHardwareByName<Joint>(tmp_str);
    LOG_DEBUG << getLabel() << "'s joint_" << count
        << ": " << tmp_str << ",\t" << jnt;
    if (nullptr == jnt) {
      LOG_WARNING << "Can't get joint '" << tmp_str
          << "' pointer from LabelSystem.";
      continue;
    }
    joints_by_type_[jnt->joint_type()] = jnt;
    jnt_cmds_[jnt->joint_type()]       = jnt->joint_command_const_pointer();
    jnt_mods_[jnt->joint_type()]       = jnt->joint_command_mode_const_pointer();
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

void LegNode::updateFromBuf(const char* __p) {
  int offset  = 0;
  for (const auto& type : {JntType::YAW, JntType::HIP, JntType::KNEE}) {
    joints_by_type_[type]->updateJointCount((__p[offset] | (__p[offset + 1] << 8)));
    offset += 2; // each count will stand two bytes.
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
  int offset = 0;
  for (const auto& type : {JntType::YAW, JntType::HIP, JntType::KNEE}) {
    if (joints_by_type_[type]->new_command_) {
      Packet cmd{node_id_, MII_MSG_COMMON_DATA_1, 6, {0}};

      memcpy(cmd.data, jnt_cmds_[type], offset + 2 * sizeof(char));
      joints_by_type_[type]->new_command_ = false;

      pkts.push_back(cmd);
    }
    offset += 2; // Each count stand two bytes.
  }

  return true;
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::LegNode, middleware::Label)
