/*
 * joint.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#include "system/foundation/cfg_reader.h"

#include <chrono>
#include <tinyxml.h>
#include <boost/algorithm/string.hpp>
#include <repository/resource/joint.h>
#include <repository/resource/joint_manager.h>
#include <system/platform/protocol/qr_protocol.h>
#include <system/foundation/utf.h>

namespace middleware {
struct JointState {
  JointState(double pos = 0, double vel = 0, double s = 0, double o = 0)
  : pos_(pos), vel_(vel), tor_(0)
  { };
  // 实际获取的数据
  double pos_;
  // 需要propagate实例中， 通过软件代码计算出速度填入数据
  double vel_;
  double tor_;
  // 计算速度的辅助变量, 保存前一次更新的时间
  std::chrono::high_resolution_clock::time_point previous_time_;
};

struct JointCommand {
  double     command_;
  JntCmdType mode_;
  JointCommand(double cmd = 0, JntCmdType mode = JntCmdType::POS)
    : /*id_(0), */command_(cmd), mode_(mode) { };
};

Joint::Joint(const MiiString& l)
  : Label(l), new_command_(false), jnt_type_(JntType::UNKNOWN_JNT),
    leg_type_(LegType::UNKNOWN_LEG), scale_(0), offset_(0), /*msg_id_(INVALID_BYTE),*/
    joint_state_(new JointState), joint_command_(new JointCommand) {
  // The code as follow should be here.
  // JointManager::instance()->add(this);
}

Joint::~Joint() {
  if (nullptr != joint_state_) {
    delete joint_state_;
    joint_state_ = nullptr;
  }
  if (nullptr != joint_command_) {
    delete joint_command_;
    joint_command_ = nullptr;
  }
}

bool Joint::init() {
  auto cfg = MiiCfgReader::instance();

  cfg->get_value_fatal(getLabel(), "jnt",  jnt_type_);
  cfg->get_value_fatal(getLabel(), "leg",  leg_type_);
  cfg->get_value_fatal(getLabel(), "name", jnt_name_);

  cfg->get_value(getLabel(), "scale",  scale_);
  cfg->get_value(getLabel(), "offset", offset_);

  JointManager::instance()->add(this);
  return true;
}

const MiiString& Joint::joint_name() const { return jnt_name_; }
const JntType& Joint::joint_type() const { return jnt_type_; }
const LegType& Joint::owner_type()   const { return leg_type_; }

void Joint::updateJointCount(short _count) {
  double pos = joint_state_->pos_;
  joint_state_->pos_ = _count * scale_ + offset_;
  auto t0 = std::chrono::high_resolution_clock::now();
  auto duration = t0 - joint_state_->previous_time_;
  auto count = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
  joint_state_->vel_ = (joint_state_->pos_ - pos) / count;
}

double Joint::joint_position() {
  return joint_state_->pos_;
}

const double& Joint::joint_position_const_ref() {
  return joint_state_->pos_;
}

const double* Joint::joint_position_const_pointer() {
  return &(joint_state_->pos_);
}

double Joint::joint_velocity() {
  return joint_state_->vel_;
}

const double& Joint::joint_velocity_const_ref() {
  return joint_state_->vel_;
}

const double* Joint::joint_velocity_const_pointer() {
  return &(joint_state_->vel_);
}

double Joint::joint_torque() {
  return joint_state_->tor_;
}

const double& Joint::joint_torque_const_ref() {
  return joint_state_->tor_;
}

const double* Joint::joint_torque_const_pointer() {
  return &(joint_state_->tor_);
}

// About joint command
void Joint::updateJointCommand(double v) {
  joint_command_->command_ = v;
  new_command_ = true;
}

void Joint::updateJointCommand(JntCmdType mode) {
  joint_command_->mode_ = mode;
}

void Joint::updateJointCommand(double v, JntCmdType t) {
  joint_command_->mode_    = t;
  joint_command_->command_ = v;
  new_command_ = true;
}

double Joint::joint_command() {
  return joint_command_->command_;
}

const double& Joint::joint_command_const_ref() {
  return joint_command_->command_;
}

const double* Joint::joint_command_const_pointer() {
  return &(joint_command_->command_);
}

JntCmdType Joint::joint_command_mode() {
  return joint_command_->mode_;
}

const JntCmdType& Joint::joint_command_mode_const_ref() {
  return joint_command_->mode_;
}
const JntCmdType* Joint::joint_command_mode_const_pointer() {
  return &(joint_command_->mode_);
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::Joint, middleware::Label)
