/*
 * joint.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#include "middleware/hardware/joint.h"
#include "middleware/util/qr_protocol.h"
#include "system/utils/cfg_reader.h"

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

Joint::Joint(MiiStringConstRef l)
  : Label(l), new_command_(false), jnt_type_(JntType::UNKNOWN_JNT),
    leg_type_(LegType::UNKNOWN_LEG), scale_(0), offset_(0), msg_id_(INVALID_ID),
    joint_state_(new JointState), joint_command_(new JointCommand) {
}

bool Joint::init() {
  auto cfg = MiiCfgReader::instance();

  bool ret = true;
  std::string tmp_str;
  cfg->get_value_fatal(getLabel(), "jnt", tmp_str);
  boost::to_lower(tmp_str);
  if (0 == tmp_str.compare("yaw")) {
    jnt_type_ = JntType::YAW;
  } else if (0 == tmp_str.compare("knee")) {
    jnt_type_ = JntType::KNEE;
  } else if (0 == tmp_str.compare("hip")) {
    jnt_type_ = JntType::HIP;
  } else {
    LOG_WARNING << "Error the 'jnt' TAG(" << tmp_str << ") in the 'joint' TAG, "
        << "require 'yaw', 'knee' or 'hip'";
    ret = false;
  }

  cfg->get_value_fatal(getLabel(), "leg", tmp_str);
  boost::to_lower(tmp_str);
  if (0 == tmp_str.compare("fl")) {
    leg_type_ = LegType::FL;
  } else if (0 == tmp_str.compare("fr")) {
    leg_type_ = LegType::FR;
  } else if (0 == tmp_str.compare("hl")) {
    leg_type_ = LegType::HL;
  } else if (0 == tmp_str.compare("hr")) {
    leg_type_ = LegType::HR;
  } else {
    LOG_WARNING << "Error the 'leg' TAG(" << tmp_str << ") in the 'joint' TAG, "
        << "require 'hl', 'fr', 'hl' or 'hr'";
    ret = false;
  }

  cfg->get_value_fatal(getLabel(), "msg_id", msg_id_);
  cfg->get_value_fatal(getLabel(), "scale",  scale_);
  cfg->get_value_fatal(getLabel(), "offset", offset_);

  return true;
}

inline const JntType& Joint::joint_type() const { return jnt_type_; }
inline const LegType& Joint::leg_type()   const { return leg_type_; }

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

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::Joint, middleware::Label)
