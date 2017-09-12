/*
 * hw_unit.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: silence
 */

#include <system/foundation/utf.h>
#include <system/platform/sw_node/sw_node_manager.h>
#include <system/platform/sw_node/sw_node.h>
#include "system/foundation/cfg_reader.h"


namespace middleware {

SWNode::SWNode(const MiiString& l)
: Label(l), node_id_(INVALID_BYTE) {
  SWNodeManager::instance()->add(this);
}

bool SWNode::init() {
  auto cfg = MiiCfgReader::instance();
  cfg->get_value_fatal(getLabel(), "node_id", node_id_);

  return true;
}

void SWNode::handleMsg(const Packet&) {
  ;
}

SWNode::~SWNode()                 { }
bool SWNode::generateCmd(MiiVector<Packet>&) { return false;  }
bool SWNode::requireCmdDeliver()  { return true;   }

} /* namespace quadruped_robot_driver */


