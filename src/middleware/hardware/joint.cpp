/*
 * joint.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#include "middleware/hardware/joint.h"
#include "middleware/util/log.h"
#include "middleware/util/proto/dragon.pb.h"

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

namespace middleware {
typedef boost::shared_ptr<Joint> JointSp;

JointCommand::JointCommand(const LegType& leg, const JntType& jnt, double cmd, JntCmdType mode)
      : /*id_(0), */command_(cmd), mode_(mode),
        leg_(leg), jnt_(jnt)
{ }

bool JointCommand::parseTo(Command* c) {
  c->set_idx(CmdType::JNT_TASK);
  auto cmd = c->mutable_jnt_cmd();
  cmd->set_leg(leg_);
  cmd->set_jnt(jnt_);
  cmd->set_type(mode_);
  cmd->set_cmd(command_);
  return true;
}

JointState::JointState(const LegType& leg, const JntType& jnt, double pos, double vel)
     : pos_(pos), vel_(vel),
       leg_(leg), jnt_(jnt)
{}

bool JointState::updateFrom(const Feedback* fb) {
  if (FbType::JOINT_STATES != fb->idx() || leg_ != fb->joint_states().leg()
      || JntType::N_JNTS != fb->joint_states().pos_size()) {
    /*LOG_ERROR << "ERROR UPDATE in " << leg_ << "leg(Type: 0 vs " << fb->idx()
        << ", Leg: " << leg_ << " vs " << fb->joint_states().leg()
        << ", pos_size: 3 vs " << fb->joint_states().pos_size() << ")";*/
    return false;
  }

  vel_ = (fb->joint_states().pos(jnt_) - pos_)
      / std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - previous_time_).count();
  pos_ = fb->joint_states().pos(jnt_);
  previous_time_ = std::chrono::high_resolution_clock::now();
  return true;
}

sensor_msgs::JointState Joint::s_joint_states_msg_;
ros::Publisher          Joint::s_joint_states_pub_;
bool                    Joint::s_ros_pub_init_ = false;

Joint::Joint(const std::string& name)
  : HwUnit(name),
    leg_(LegType::FL), jnt_(JntType::HIP),
    joint_state_(nullptr), joint_command_(nullptr)
{ }

void Joint::publish() {
  if (composite_map_.empty()) return;

  if (0 == s_joint_states_msg_.name.size()) {
    s_joint_states_msg_.name.reserve(composite_map_.size());
    s_joint_states_msg_.position.reserve(composite_map_.size());
    s_joint_states_msg_.velocity.reserve(composite_map_.size());
    s_joint_states_msg_.effort.reserve(composite_map_.size());
  }

  s_joint_states_msg_.name.clear();
  s_joint_states_msg_.position.clear();
  s_joint_states_msg_.velocity.clear();
  s_joint_states_msg_.effort.clear();
  for (auto& j : composite_map_) {
    JointSp jnt = boost::dynamic_pointer_cast<Joint>(j.second);
    s_joint_states_msg_.name.push_back(jnt->hw_name_);
    s_joint_states_msg_.position.push_back(jnt->joint_state_->pos_);
    s_joint_states_msg_.velocity.push_back(jnt->joint_state_->vel_);
    s_joint_states_msg_.effort.push_back(0);
  }

  s_joint_states_msg_.header.stamp = ros::Time::now();
  s_joint_states_pub_.publish(s_joint_states_msg_);
}

bool Joint::init(TiXmlElement* root) {
  if (!root) {
    LOG_ERROR << "The format of 'joint' tag is wrong!";
    return false;
  } else if ((root->Attribute("leg")) && (root->Attribute("jnt"))) {
    // Component
    return initComponent(root);
  } else {
    // Composite
    return initComposite(root);
  }
}

bool Joint::initComposite(TiXmlElement* root) {
  joint_state_.reset();
  joint_command_.reset();
  if (!root->Attribute("name"))
    root->SetAttribute("name", "Joints");
  hw_name_ = root->Attribute("name");

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

  if (!s_ros_pub_init_) {
    ros::NodeHandle nh;
    s_joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    s_ros_pub_init_     = true;
  }

  LOG_INFO << "[Joints: '" << root->Attribute("name")
      << "'] initialize end, create the rest 'joint' instance...";

  for (auto j = root->FirstChildElement("joint");
        j != nullptr; j = j->NextSiblingElement("joint")) {
    if (!j->Attribute("name"))
      j->SetAttribute("name", "joint_" + std::to_string(composite_map_.size()));
    if (!j->Attribute("channel"))
      j->SetAttribute("channel", root->Attribute("channel"));

    HwUnit* unit = new Joint();
    unit->init(j);
    this->add(unit->hw_name_, unit);
  }

  return true;
}
bool Joint::initComponent(TiXmlElement* root) {
  LOG_INFO << "[Joint: '" << root->Attribute("name") << "'] initialize start...";
  std::string tmp_str = root->Attribute("leg");
  boost::to_lower(tmp_str);
  if (0 == tmp_str.compare("fl")) {
    leg_ = LegType::FL;
  } else if (0 == tmp_str.compare("fr")) {
    leg_ = LegType::FR;
  } else if (0 == tmp_str.compare("hl")) {
    leg_ = LegType::HL;
  } else if (0 == tmp_str.compare("hr")) {
    leg_ = LegType::HR;
  } else {
    LOG_ERROR << "Error the 'leg' TAG(" << tmp_str << ") in the 'joint' TAG, "
        << "require 'fl', 'fr', 'hl' or 'hr'";
    return false;
  }

  tmp_str = root->Attribute("jnt");
  boost::to_lower(tmp_str);
  if (0 == tmp_str.compare("yaw")) {
    jnt_ = JntType::YAW;
  } else if (0 == tmp_str.compare("knee")) {
    jnt_ = JntType::KNEE;
  } else if (0 == tmp_str.compare("hip")) {
    jnt_ = JntType::HIP;
  } else {
    LOG_ERROR << "Error the 'jnt' TAG(" << tmp_str << ") in the 'joint' TAG, "
        << "require 'yaw', 'knee' or 'hip'";
    return false;
  }

  joint_state_.reset(new StateType(leg_, jnt_));
  joint_command_.reset(new CmdType(leg_, jnt_));

  if (!s_ros_pub_init_) {
    ros::NodeHandle nh;
    s_joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    s_ros_pub_init_     = true;
  }

  LOG_INFO << "[Joint: '" << root->Attribute("name")
      << "'] initialize end, call HwUnit::init()...";
  return HwUnit::init(root);
}

HwStateSp Joint::getStateHandle() { return joint_state_; }
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
void Joint::setState(const HwState& s) {
  LOG_WARNING << "The joint is not assigned state!";
}

void Joint::setCommand(const HwCommand& c) {
  const CmdType& motor_cmd = dynamic_cast<const CmdType&>(c);
  JntCmdType mode = motor_cmd.mode_;
  joint_command_->mode_ = mode;
  double val = motor_cmd.command_;
  joint_command_->command_ = val;
}

void Joint::check() {
  if (!composite_map_.empty()) {
    for (auto& ele : composite_map_) {
      ele.second->check();
    }
  } else {
    LOG_WARNING << "=============CHECK=============";
    LOG_INFO << "NAME\tCMD_C\tSTATE_C";
    LOG_INFO << hw_name_ << "\t" << cmd_channel_ << "\t" << state_channel_;
    LOG_INFO << "TYPE\tADDR\tCOUNT";
    LOG_INFO << "STATE\t" << joint_state_.get() << "\t" << joint_state_.use_count();
    LOG_INFO << "COMMAND\t" << joint_command_.get() << "\t" << joint_command_.use_count();
    LOG_WARNING << "===============================";
  }
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(middleware::Joint, middleware::HwUnit)
