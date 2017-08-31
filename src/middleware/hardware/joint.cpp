/*
 * joint.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#include "middleware/hardware/joint.h"
#include "middleware/util/qr_protocol.h"

#include <chrono>
#include <tinyxml.h>
#include <boost/algorithm/string.hpp>
#include <system/utils/log.h>

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
  : Label(root->Attribute("name")),
    new_command_(false), scale_(0), offset_(0), msg_id_(INVALID_ID),
    joint_state_(new JointState), joint_command_(new JointCommand) {
  LOG_INFO << "[Joint: '" << root->Attribute("name") << "'] initialize start...";
  if ((!root->Attribute("leg")) || (!root->Attribute("jnt")) || (!root->Attribute("msg_id"))) {
    LOG_ERROR << "The format of 'joint' tag is wrong! An example:";
    LOG_ERROR << "<joint name=\"joint_name\"  leg=\"fl\" jnt=\"yaw\"  id=\"0x01\"  />";
    return;
  }

  /*std::string tmp_str = root->Attribute("jnt");
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
  }*/
  std::string tmp_str = root->Attribute("msg_id");
  std::string template_str;
  if (('0' == tmp_str[0]) && ('x' == tmp_str[1])) {
    // Hex to id
    template_str = "0x%x";
  } else {
    template_str = "%d";
  }
  unsigned int id;
  sscanf(tmp_str.c_str(), template_str.c_str(), &id);
  msg_id_ = id;

  root->Attribute("scale",  &scale_);
  root->Attribute("offset", &offset_);
}

inline void Joint::updateJointPosition(short _count) {
  double pos = _count * scale_ + offset_;
  /*auto t = std::chrono::high_resolution_clock::now();
  auto duration = t - joint_state_->previous_time_;
  auto count = std::chrono::duration_cast<std::chrono::duration<double>>(duration.count());
  joint_state_->vel_ = (pos - joint_state_->pos_) / count;*/
}

inline double Joint::joint_position() {
  return joint_state_->pos_;
}

inline double Joint::joint_velocity() {
  return joint_state_->vel_;
}

inline double Joint::joint_torque() {
  return joint_state_->tor_;
}

// About joint command
inline void Joint::updateJointCommand(double v) {
  joint_command_->command_ = v;
  new_command_ = true;
}

inline void Joint::updateJointCommand(double v, JntCmdType t) {
  joint_command_->mode_    = t;
  joint_command_->command_ = v;
  new_command_ = true;
}

inline double Joint::joint_command() {
  return joint_command_->command_;
}

inline void Joint::changeJointCommandMode(JntCmdType mode) {
  joint_command_->mode_ = mode;
}

inline JntCmdType Joint::joint_command_mode() {
  return joint_command_->mode_;
}

inline bool Joint::new_command(Packet* pkt) {
  if (!new_command_) return false;
  new_command_ = false;

  pkt->msg_id = msg_id_;
  pkt->size   = 2;
  short count = (joint_command_->command_ - offset_) / scale_;
  pkt->data[0] = count;
  pkt->data[1] = count >> 8;
  // memcpy(pkt->data, &count, sizeof(short));
  return false;
}

} /* namespace middleware */
