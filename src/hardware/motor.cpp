/*
 * actuator.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include "middleware/hardware/motor.h"
#include "middleware/util/log.h"


namespace middleware {

MotorState::MotorState(double pos, double vel, double tor)
    : pos_(pos), vel_(vel), tor_(tor)
{ }

MotorState::~MotorState()
{ }

MotorCmd::MotorCmd(LegType leg, JntType jnt, double cmd, JntCmdType mode)
    : leg_(leg), jnt_(jnt),
      command_(cmd), mode_(mode)
{ }

MotorCmd::~MotorCmd()
{ }

bool MotorCmd::update(Command* cmd) {
  cmd->set_idx(CmdType::JNT_TASK);
  JntCmd* jnt = cmd->mutable_jnt_cmd();
  jnt->set_leg(leg_);
  jnt->set_jnt(jnt_);
  jnt->set_type(mode_);
  jnt->set_cmd(command_);

  return true;
}


Motor::Motor(const std::string&  name, JntCmdType mode)
    : HwUnit(name),
      motor_state_(new StateType()),
      motor_cmd_(nullptr)
{ }

Motor::~Motor()
{ }

bool Motor::init(TiXmlElement* para) {
  if (nullptr == para->Attribute("name")) {
    LOG_ERROR << "Can't found the 'name' TAG in the 'parameter' TAG";
    return false;
  }
  hw_name_ = para->Attribute("name");
  std::string tmp_str = "";
  JntCmdType mode = JntCmdType::POS;
  if (nullptr != para->Attribute("mode")) {
    tmp_str = para->Attribute("mode");
  } else {
    LOG_ERROR << "Can't found the 'mode' TAG in the 'parameter' TAG";
    return false;
  }
  if (0 == tmp_str.compare("velocity")) {
    mode = JntCmdType::VEL;
  } else if (0 == tmp_str.compare("torque")) {
    mode = JntCmdType::TOR;
  } else {
    LOG_ERROR << "Error the 'mode' TAG in the 'parameter' TAG";
    return false;
  }
  tmp_str = "";
  LegType leg;
  if (nullptr != para->Attribute("leg")) {
    tmp_str = para->Attribute("leg");
  } else {
    LOG_ERROR << "Can't found the 'leg' TAG in the 'parameter' TAG";
    return false;
  }
  if (0 == tmp_str.compare("fl") || 0 == tmp_str.compare("FL")) {
    leg = LegType::FL;
  } else if (0 == tmp_str.compare("fr") || 0 == tmp_str.compare("FR")) {
    leg = LegType::FR;
  } else if (0 == tmp_str.compare("hl") || 0 == tmp_str.compare("HL")) {
    leg = LegType::HL;
  } else if (0 == tmp_str.compare("hr") || 0 == tmp_str.compare("HR")) {
    leg = LegType::HR;
  } else {
    LOG_ERROR << "Error the 'leg' TAG in the 'parameter' TAG";
    return false;
  }
  tmp_str = "";
  JntType jnt;
  if (nullptr != para->Attribute("jnt")) {
    tmp_str = para->Attribute("jnt");
  } else {
    LOG_ERROR << "Can't found the 'jnt' TAG in the 'parameter' TAG";
    return false;
  }
  if (0 == tmp_str.compare("yaw") || 0 == tmp_str.compare("YAW")) {
    jnt = JntType::YAW;
  } else if (0 == tmp_str.compare("knee") || 0 == tmp_str.compare("KNEE")) {
    jnt = JntType::KNEE;
  } else if (0 == tmp_str.compare("hip") || 0 == tmp_str.compare("HIP")) {
    jnt = JntType::HIP;
  } else {
    LOG_ERROR << "Error the 'jnt' TAG in the 'parameter' TAG";
    return false;
  }

  double val = 0;
  if (nullptr != para->Attribute("value")) {
    std::stringstream ss;
    ss << para->Attribute("value");
    ss >> val;
  }
  motor_cmd_.reset(new CmdType(leg, jnt, val, mode));
  return true;
}

HwStateSp Motor::getStataHandle() {
  return motor_state_;
}

HwCmdSp Motor::getCmdHandle() {
  return motor_cmd_;
}

HwStateSp Motor::getState() {
  return HwStateSp(new StateType(
      motor_state_->pos_, motor_state_->vel_, motor_state_->tor_));
}

HwCmdSp Motor::getCommand() {
  return HwCmdSp(new CmdType(motor_cmd_->leg_, motor_cmd_->jnt_,
      motor_cmd_->command_, motor_cmd_->mode_));
}

void Motor::setState(const HwState& state) {
  const StateType& motor_state = dynamic_cast<const StateType&>(state);
  double val = motor_state.pos_;
  motor_state_->pos_ = val;
  val = motor_state.vel_;
  motor_state_->vel_ = val;
  val = motor_state.tor_;
  motor_state_->tor_ = val;
}

void Motor::setCommand(const HwCommand& cmd) {
  const CmdType& motor_cmd = dynamic_cast<const CmdType&>(cmd);
  JntCmdType mode = motor_cmd.mode_;
  motor_cmd_->mode_ = mode;
  double val = motor_cmd.command_;
  motor_cmd_->command_ = val;
}

void Motor::check() {
  LOG_WARNING << "================check================";
  LOG_INFO << "NAME: " << hw_name_;
  // LOG_WARNING << "\t-------------------------------------";
  LOG_INFO << "TYPE\tADDR\tCOUNT";
  LOG_INFO << "STATE\t" << motor_state_.get() << "\t" << motor_state_.use_count();
  LOG_INFO << "COMMAND\t" << motor_cmd_.get() << "\t" << motor_cmd_.use_count();
  LOG_WARNING << "=====================================";
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::Motor, middleware::HwUnit)
