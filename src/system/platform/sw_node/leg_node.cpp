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

const size_t JNT_CMD_DATA_SIZE = 6;

// angle = \frac{360 \pi \alpha}{180*4096} C - \frac{\pi}{18000}\alpha*\beta
// so, the ABS(scale) = \frac{360 \pi \alpha}{180*4096} = \frac{360\pi}{180*4096}
// offset = - \frac{\pi}{18000}\alpha*\beta = -0.000174528*\beta
struct __PrivateLinearParams {
  double scale;
  double offset;
};

LegNode::LegNode(const MiiString& __l)
  : SWNode(__l), leg_(LegType::UNKNOWN_LEG), td_(nullptr) {
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
  cfg->get_value_fatal(getLabel(), "leg", leg_);

  int count = 0;
  jnts_by_type_.resize(JntType::N_JNTS);
  jnt_params_.resize(JntType::N_JNTS);
  MiiString tag = Label::make_label(getLabel(), "joint_0");
  MiiString tmp_str;
  while(cfg->get_value(tag, "label", tmp_str)) {
    Joint* jnt = Label::getHardwareByName<Joint>(tmp_str);
    // LOG_DEBUG << getLabel() << "'s joint_" << count
    //     << ": " << tmp_str << ",\t" << jnt;
    if (nullptr == jnt) {
      LOG_WARNING << "Can't get joint '" << tmp_str
          << "' pointer from LabelSystem.";
      tag = Label::make_label(getLabel(), "joint_" + std::to_string(++count));
      continue;
    }
    jnts_by_type_[jnt->joint_type()] = jnt;

    auto param  = new __PrivateLinearParams;
    double alpha, beta = 0;
    cfg->get_value_fatal(tag, "scale",  alpha);
    cfg->get_value_fatal(tag, "offset", beta);
    param->scale  = alpha * 0.001533981;
    param->offset = alpha * beta * -0.000174528;
    jnt_params_[jnt->joint_type()]  = param;

    jnt_cmds_[jnt->joint_type()]       = jnt->joint_command_const_pointer();
    jnt_mods_[jnt->joint_type()]       = jnt->joint_command_mode_const_pointer();

    tag = Label::make_label(getLabel(), "joint_" + std::to_string(++count));
  }

  tag = Label::make_label(getLabel(), "touchdown");
  if ((cfg->get_value(tag, "label", tmp_str))
      && (td_ = Label::getHardwareByName<ForceSensor>(tmp_str))) {
    // LOG_DEBUG << getLabel() << "'s TD: " << td_->getLabel() << "\t" << td_;
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

    // angle = \frac{360 \pi \alpha}{180*4096} C - \frac{\pi}{18000}\alpha*\beta
    // so, the ABS(scale) = \frac{360 \pi \alpha}{180*4096} = \frac{360\pi}{180*4096}
    // offset = - \frac{\pi}{18000}\alpha*\beta = -0.000174528*\beta
    pos = jnt_params_[type]->scale * (double)count + jnt_params_[type]->offset;
    
    jnts_by_type_[type]->updateJointPosition(pos);
    offset += sizeof(count); // each count will stand two bytes.
  }

  // if (LegType::HL == leg_) printf("%d: 0x%02X, 0x%02X", leg_, __p[offset], __p[offset + 1]);
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

bool LegNode::generateCmd(MiiVector<Packet>& pkts) {
  int offset  = 0;
  short count = 0;
  bool is_any_valid = false;
  Packet cmd{INVALID_BYTE, node_id_, MII_MSG_COMMON_DATA_1, JNT_CMD_DATA_SIZE, {0}};

  // TODO Judge the mode of command
  // switch (*jnt_mods_[])
  for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
    if (jnts_by_type_[type]->new_command_) {
      is_any_valid = true;
      count = (*jnt_cmds_[type] - jnt_params_[type]->offset) / jnt_params_[type]->scale;
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
CLASS_LOADER_REGISTER_CLASS(middleware::LegNode, Label)
