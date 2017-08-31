/*
 * hw_unit.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: silence
 */

#include <middleware/util/proto/dragon.pb.h>
#include <system/utils/log.h>
#include "system/utils/cfg_reader.h"
#include "middleware/hardware/hw_unit.h"
#include "middleware/hardware/hw_manager.h"


namespace middleware {

HwUnit::HwUnit(MiiStringConstRef l)
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
bool HwUnit::generateCmd(std::vector<Packet>&) { return false;  }
bool HwUnit::requireCmdDeliver()  { return true;   }

} /* namespace quadruped_robot_driver */


