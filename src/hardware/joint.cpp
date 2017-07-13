/*
 * joint.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#include "middleware/hardware/joint.h"
#include "middleware/util/log.h"



namespace middleware {
typedef boost::shared_ptr<Joint> JointSp;

JointCommand::JointCommand(const LegType& leg, const JntType& jnt, double cmd, JntCmdType mode)
      : command_(cmd), mode_(mode),
        leg_(leg), jnt_(jnt)
{}

bool JointCommand::parseTo(class Command*) {
  // TODO
  return true;
}

JointState::JointState(const LegType& leg, const JntType& jnt, double pos, double vel)
     : pos_(pos), vel_(vel),
       leg_(leg), jnt_(jnt)
{}

bool JointState::updateFrom(const class Feedback*) {
  // TODO
  return true;
}

sensor_msgs::JointState* Joint::s_joint_states_msg_ = nullptr;
ros::Publisher*          Joint::s_joint_states_pub_ = nullptr;

Joint::Joint(const std::string& name)
  : HwUnit(name),
    leg_(LegType::FL), jnt_(JntType::HIP),
    joint_state_(nullptr), joint_command_(nullptr)
{ }

void Joint::publish() {
  if (composite_map_.empty()) return;
  if (0 == s_joint_states_msg_->name.size()) {
    s_joint_states_msg_->name.reserve(composite_map_.size());
    s_joint_states_msg_->position.reserve(composite_map_.size());
    s_joint_states_msg_->velocity.reserve(composite_map_.size());
    s_joint_states_msg_->effort.reserve(composite_map_.size());
  }

  s_joint_states_msg_->name.clear();
  s_joint_states_msg_->position.clear();
  s_joint_states_msg_->velocity.clear();
  s_joint_states_msg_->effort.clear();
  for (auto& j : composite_map_) {
    JointSp jnt = boost::dynamic_pointer_cast<Joint>(j.second);
    s_joint_states_msg_->name.push_back(jnt->hw_name_);
    s_joint_states_msg_->position.push_back(jnt->joint_state_->pos_);
    s_joint_states_msg_->velocity.push_back(jnt->joint_state_->vel_);
    s_joint_states_msg_->effort.push_back(0);
  }

  s_joint_states_msg_->header.stamp = ros::Time::now();
  s_joint_states_pub_->publish(*s_joint_states_msg_);
}

bool Joint::init(TiXmlElement* root) {
  if ((!root) || (!root->Attribute("name"))
      || (!root->Attribute("channel"))
      || (!root->Attribute("leg"))
      || (!root->Attribute("jnt"))) {
    LOG_ERROR << "The format of 'joint' tag is wrong!";
    return false;
  }

  std::string tmp_str = root->Attribute("leg");
  if (0 == tmp_str.compare("fl") || 0 == tmp_str.compare("FL")) {
    leg_ = LegType::FL;
  } else if (0 == tmp_str.compare("fr") || 0 == tmp_str.compare("FR")) {
    leg_ = LegType::FR;
  } else if (0 == tmp_str.compare("hl") || 0 == tmp_str.compare("HL")) {
    leg_ = LegType::HL;
  } else if (0 == tmp_str.compare("hr") || 0 == tmp_str.compare("HR")) {
    leg_ = LegType::HR;
  } else {
    LOG_ERROR << "Error the 'leg' TAG in the 'joint' TAG";
    return false;
  }

  tmp_str = root->Attribute("jnt");
  if (0 == tmp_str.compare("yaw") || 0 == tmp_str.compare("YAW")) {
    jnt_ = JntType::YAW;
  } else if (0 == tmp_str.compare("knee") || 0 == tmp_str.compare("KNEE")) {
    jnt_ = JntType::KNEE;
  } else if (0 == tmp_str.compare("hip") || 0 == tmp_str.compare("HIP")) {
    jnt_ = JntType::HIP;
  } else {
    LOG_ERROR << "Error the 'jnt' TAG in the 'parameter' TAG";
    return false;
  }

  joint_state_.reset(new StateType(leg_, jnt_));
  joint_command_.reset(new CmdType(leg_, jnt_));
  hw_name_ = root->Attribute("name");

  if (!s_joint_states_pub_) {
    // TODO ros::NodeHandle ;
  }

  return true;
}

HwStateSp Joint::getStataHandle() { return joint_state_; }
HwCmdSp Joint::getCmdHandle()     { return joint_command_; }

HwStateSp Joint::getState() {
  return HwStateSp(new StateType(joint_state_->leg_, joint_state_->jnt_,
      joint_state_->pos_, joint_state_->vel_));
}

HwCmdSp Joint::getCommand() {
  return HwCmdSp(new CmdType(joint_command_->leg_, joint_command_->jnt_,
      joint_command_->command_, joint_command_->mode_));
}

/// 不能设置关节状态
void Joint::setState(const HwState& s) {  }

void Joint::setCommand(const HwCommand& c) {
  const CmdType& motor_cmd = dynamic_cast<const CmdType&>(c);
  JntCmdType mode = motor_cmd.mode_;
  joint_command_->mode_ = mode;
  double val = motor_cmd.command_;
  joint_command_->command_ = val;
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(middleware::Joint, middleware::HwUnit)
