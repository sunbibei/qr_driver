/*
 * leg_node.cpp
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#include "system/foundation/cfg_reader.h"
#include <boost/algorithm/string.hpp>
#include <system/resources/joint.h>
#include <system/resources/touchdown.h>
#include <system/foundation/utf.h>
#include <system/platform/sw_node/leg_node.h>

#include <iomanip>

namespace middleware {

LegNode::LegNode(const MiiString& __l)
  : SWNode(__l), leg_(LegType::UNKNOWN_LEG), td_(nullptr) {
}

LegNode::~LegNode() {
  for (auto& jnt : joints_) {
    delete jnt;
    jnt = nullptr;
  }

  delete td_;
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
  unsigned char max_id = 0x00;
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
    joints_.push_back(jnt);
    if (max_id < jnt->msg_id_) max_id = jnt->msg_id_;
  }
  if (0x00 == max_id) {
    LOG_WARNING << "No joint push back!";
    return false;
  }

  joints_by_id_.resize(++max_id);
  for (auto j : joints_)
    joints_by_id_[j->msg_id_] = j;

  tag = Label::make_label(getLabel(), "touchdown");
  if ((cfg->get_value(tag, "label", tmp_str))
      && (td_ = Label::getHardwareByName<TouchDown>(tmp_str))) {
    LOG_DEBUG << getLabel() << "'s TD: " << td_->getLabel() << "\t" << td_;
    return true;
  } else {
    LOG_WARNING << "The touchdown parameter is not found.";
    return false;
  }
}

void LegNode::handleMsg(const Packet& pkt) {
  if (pkt.node_id != node_id_) {
    LOG_ERROR << "Wrong match id between Packet and Joint";
    return;
  }

  short count = pkt.data[0] + (pkt.data[1] << 8);
  // memcpy(&count , pkt.data, sizeof(short));
  if (pkt.msg_id == td_->msg_id_)
    td_->updateTouchdownState(count);
  else
    joints_by_id_[pkt.msg_id]->updateJointPosition(count);
}

bool LegNode::generateCmd(std::vector<Packet>& pkts) {
  Packet cmd;
  cmd.node_id = node_id_;
  for (auto& j : joints_) {
    // if (j->new_command(&cmd))
      continue;

    // double new_cmd = j->joint_command();
  }

  return true;
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::LegNode, middleware::Label)
