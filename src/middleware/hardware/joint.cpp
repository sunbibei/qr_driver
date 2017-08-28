/*
 * joint.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#include "middleware/hardware/joint.h"
#include "middleware/util/log.h"
#include "middleware/util/proto/dragon.pb.h"
#include "middleware/util/qr_protocol.h"

#include <chrono>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

namespace middleware {
struct JointState {
  JointState(double pos = 0, double vel = 0)
  : pos_(pos), vel_(vel) { };
  // 实际获取的数据
  double pos_;
  // 需要propagate实例中， 通过软件代码计算出速度填入数据
  double vel_;
private:
  // 计算速度的辅助变量, 保存前一次更新的时间
  std::chrono::high_resolution_clock::time_point previous_time_;
};

struct JointCommand {
  double     command_;
  JntCmdType mode_;
  JointCommand(double cmd = 0, JntCmdType mode = JntCmdType::POS)
    : /*id_(0), */command_(cmd), mode_(mode) { };
};

Joint::Joint(TiXmlElement* root)
  : leg_(LegType::FL), jnt_(JntType::HIP), cmd_id_(INVALID_ID),
    joint_state_(new JointState), joint_command_(new JointCommand) {
  LOG_INFO << "[Joint: '" << root->Attribute("name") << "'] initialize start...";
  if ((!root->Attribute("leg")) || (!root->Attribute("jnt")) || (!root->Attribute("cmd_id"))) {
    LOG_ERROR << "The format of 'joint' tag is wrong! An example:";
    LOG_ERROR << "<joint name=\"joint_name\"  leg=\"fl\" jnt=\"yaw\"  id=\"0x01\"  />";
    return;
  }

  std::string tmp_str = root->Attribute("jnt");
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
  }
  tmp_str = root->Attribute("msg_id");
  std::string template_str;
  if (('0' == tmp_str[0]) && ('x' == tmp_str[1])) {
    // Hex to id
    template_str = "0x%x";
  } else {
    template_str = "%d";
  }
  unsigned int id;
  sscanf(tmp_str.c_str(), template_str.c_str(), &id);
  cmd_id_ = id;
}

/*bool Joint::initComposite(TiXmlElement* root) {
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
}*/
/*bool Joint::initComponent(TiXmlElement* root) {
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

  if (!s_ros_pub_init_) {
    ros::NodeHandle nh;
    s_joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    s_ros_pub_init_     = true;
  }

  LOG_INFO << "[Joint: '" << root->Attribute("name")
      << "'] initialize end, call HwUnit::init()...";
  return HwUnit::init(root);
}*/

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::Joint, middleware::HwUnit)
