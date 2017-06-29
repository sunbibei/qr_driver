/*
 * hw_unit.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: silence
 */

#include "middleware/hardware/hw_unit.h"

#include "middleware/util/log.h"

namespace middleware {

HwUnit::HwUnit() : HwUnit("") { ; }

HwUnit::HwUnit(const std::string& name)
    : hw_name_(name) { };

HwUnit::~HwUnit() { };
void HwUnit::check() { ; }

bool HwUnit::init(TiXmlElement*)        { return true; }
HwStateSp HwUnit::getStataHandle()      { return nullptr; }
HwCmdSp HwUnit::getCmdHandle()          { return nullptr; }
HwUnit::StateTypeSp HwUnit::getState()  { return nullptr; }
HwUnit::CmdTypeSp HwUnit::getCommand()  { return nullptr; }
void HwUnit::setState(const StateType&) { return; }
void HwUnit::setCommand(const CmdType&) { return; }

} /* namespace quadruped_robot_driver */


