/*
 * hw_unit.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: silence
 */

#include <system/platform/protocol/proto/dragon.pb.h>
#include <system/platform/hw_manager.h>
#include <system/foundation/utf.h>
#include "system/foundation/cfg_reader.h"
#include "system/platform/hw_unit/hw_unit.h"


namespace middleware {

HwUnit::HwUnit(const MiiString& l)
: Label(l), node_id_(INVALID_ID) {
  HwManager::instance()->add(this);
}

bool HwUnit::init() {
  auto cfg = MiiCfgReader::instance();
  cfg->get_value_fatal(getLabel(), "node_id", node_id_);

  return true;
}

void HwUnit::handleMsg(const Packet&) {
  ;
}

HwUnit::~HwUnit()                 { }
bool HwUnit::generateCmd(MiiVector<Packet>&) { return false;  }
bool HwUnit::requireCmdDeliver()  { return true;   }

} /* namespace quadruped_robot_driver */


