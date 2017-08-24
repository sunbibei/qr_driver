/*
 * hw_unit.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: silence
 */

#include <middleware/util/proto/dragon.pb.h>
#include "middleware/hardware/hw_unit.h"
#include "middleware/middleware.h"
#include "middleware/util/log.h"


namespace middleware {

HwUnit::HwUnit() : HwUnit("") { ; }

HwUnit::HwUnit(const std::string& name)
    : hw_name_(name) { };

HwUnit::~HwUnit() { };
void HwUnit::check() { }

bool HwUnit::init(TiXmlElement* root) {
  if ((!root) || (!root->Attribute("name"))) {
    LOG_ERROR << "The format of 'joint' tag is wrong!";
    return false;
  }
  if (!root->Attribute("channel")
      && (root->Attribute("cmd_channel") || root->Attribute("state_channel"))) {
    LOG_ERROR << "The format of 'joint' tag is wrong!";
    return false;
  }

  if (root->Attribute("channel")) {
    cmd_channel_   = root->Attribute("channel");
    state_channel_ = root->Attribute("channel");
  } else {
    cmd_channel_   = root->Attribute("cmd_channel");
    state_channel_ = root->Attribute("state_channel");
  }

  hw_name_ = root->Attribute("name");

  if (requireCmdReg())
    Middleware::instance()->propagate_[cmd_channel_]
                          ->registerHandle(hw_name_, getCmdHandle());
  if (requireStateReg())
    Middleware::instance()->propagate_[state_channel_]
                          ->registerHandle(hw_name_, getStateHandle());
  return true;
}

bool                HwUnit::requireStateReg()  { return true;   }
bool                HwUnit::requireCmdReg()    { return true;   }
HwStateSp           HwUnit::getStateHandle()   { return nullptr; }
HwCmdSp             HwUnit::getCmdHandle()     { return nullptr; }
HwUnit::StateTypeSp HwUnit::getState()         { return nullptr; }
HwUnit::CmdTypeSp   HwUnit::getCommand()       { return nullptr; }

void                HwUnit::setState(const StateType&) { return; }
void                HwUnit::setCommand(const CmdType&) { return; }

void                HwUnit::publish() { }

} /* namespace quadruped_robot_driver */


