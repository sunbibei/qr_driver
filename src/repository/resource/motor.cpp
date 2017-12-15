/*
 * motor.cpp
 *
 *  Created on: Dec 15, 2017
 *      Author: robot
 */

#include <repository/resource/motor.h>
#include <repository/resource/joint.h>

namespace middleware {

struct MotorState {
  short position;
  short velocity;
  short torque;
};

struct MotorCommand {
  MotorCommand(const JntCmdType& _mode)
    : command(0), mode(_mode) {
    ;
  }

  short             command;
  const JntCmdType& mode;
};

Motor::Motor(const MiiString& _l, const JntCmdType& _mode)
  : Label(_l), joint_handle_(nullptr),
    motor_state_(new MotorState), motor_cmd_(new MotorCommand(_mode)),
    new_command_(false) {
  ;
}
bool Motor::init() {
  // TODO
  return true;
}

Motor::~Motor() {
  if (motor_state_) delete motor_state_;
  if (motor_cmd_)   delete motor_cmd_;
  motor_state_ = nullptr;
  motor_cmd_   = nullptr;
}

void Motor::updateMotorCommand(double) {
  ;
}

const MiiString&  Motor::motor_name() const {
  return joint_handle_->joint_name();
}

const MiiString&  Motor::joint_name() const {
  return joint_handle_->joint_name();
}

const JntType&    Motor::joint_type() const {
  return joint_handle_->joint_type();
}

const LegType&    Motor::leg_type()   const {
  return joint_handle_->owner_type();
}

const JntCmdType& Motor::cmd_mode()   const {
  return motor_cmd_->mode;
}

///! About the state of motor
const double  Motor::motor_position() const {
  return motor_state_->position;
}

const double& Motor::motor_position_const_ref() const {
  return motor_state_->position;
}

const double* Motor::motor_position_const_pointer() const {
  return &motor_state_->position;
}


const double  Motor::motor_velocity() const {
  return motor_state_->velocity;
}

const double& Motor::motor_velocity_const_ref() const {
  return motor_state_->velocity;
}

const double* Motor::motor_velocity_const_pointer() const {
  return &motor_state_->velocity;
}


const double  Motor::motor_torque() const {
  return motor_state_->torque;
}

const double& Motor::motor_torque_const_ref() const {
  return motor_state_->torque;
}

const double* Motor::motor_torque_const_pointer() const {
  return &motor_state_->torque;
}


///! About the command of motor
const double  Motor::motor_command() const {
  return motor_cmd_->command;
}

const double& Motor::motor_command_const_ref() const {
  return motor_cmd_->command;
}

const double* Motor::motor_command_const_pointer() const {
  return &motor_cmd_->command;
}



} /* namespace middleware */
