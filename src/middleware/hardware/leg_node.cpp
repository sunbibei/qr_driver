/*
 * leg_node.cpp
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#include "leg_node.h"
#include "touchdown.h"
#include "joint.h"
#include "middleware/util/log.h"

#include <boost/algorithm/string.hpp>

namespace middleware {

LegNode::LegNode()
  : leg_(LegType::FL), td_(nullptr) {
}

LegNode::~LegNode() {
  for (auto& jnt : joints_) {
    delete jnt;
    jnt = nullptr;
  }

  delete td_;
  td_ = nullptr;
}

bool LegNode::init(TiXmlElement* root) {
  if (!HwUnit::init(root))     return false;
  if (!root->Attribute("leg")) return false;

  std::string tmp_str = root->Attribute("leg");
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

  unsigned char max_id = 0x00;
  for (auto sub = root->FirstChildElement("joint");
      sub != nullptr; sub = sub->NextSiblingElement("joint")) {
    Joint* jnt = new Joint(sub);
    joints_.push_back(jnt);
    if (max_id < jnt->cmd_id_) max_id = jnt->cmd_id_;
  }
  auto sub = root->FirstChildElement("touchdown");
  if (nullptr == sub) return false;
  td_ = new TouchDown(sub);

  return true;
}

void LegNode::handleMsg(const Packet& pkt) {
  if (pkt.node_id != node_id_) {
    LOG_ERROR << "Wrong match id between Packet and Joint";
    return;
  }

  switch (pkt.msg_id) {
  case MII_FB_JOINT_POS:
  {
    joint_state_->pos_ = 0;
    break;
  }
  case MII_FB_JOINT_VEL: break;
  case MII_FB_JOINT_TOR: break;
  default:
    HwUnit::handleMsg(pkt);
    break;
  }
}

} /* namespace middleware */
