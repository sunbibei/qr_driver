/*
 * power_node.cpp
 *
 *  Created on: Nov 14, 2017
 *      Author: bibei
 */

#include <system/platform/sw_node/power_node.h>
#include "system/foundation/cfg_reader.h"
#include "repository/resource/power.h"

namespace middleware {

PowerNode::PowerNode(const MiiString& __l)
  : SWNode(__l), power_info_(nullptr) {

}

bool PowerNode::init() {
  if (!SWNode::init()) return false;
  auto cfg = MiiCfgReader::instance();

  MiiString power_label;
  cfg->get_value_fatal(getLabel(), "label", power_label);
  power_info_ = Label::getHardwareByName<Power>(power_label);
  if (nullptr == power_info_) {
    LOG_ERROR << "Can't get power object '" << power_label << "'";
  }

  return true;;
}

PowerNode::~PowerNode() {
  ;
}

void PowerNode::handleMsg(const Packet& pkt) {
  if (pkt.node_id != node_id_) {
    LOG_ERROR << "Wrong match id between Packet and Joint";
    return;
  }

  switch (pkt.msg_id) {
  case MII_MSG_HEARTBEAT_MSG_1:
    // parse the joint state and touchdown data
    // updateFromBuf(pkt.data);
    break;
  default:
    SWNode::handleMsg(pkt);
  }
}

bool PowerNode::requireCmdDeliver() {
  return false;
}

} /* namespace middleware */
