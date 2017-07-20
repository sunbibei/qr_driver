/*
 * joint.h
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_

#include "hw_unit.h"

#include <atomic>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace middleware {

struct JointState : public HwState {
  // 实际获取的数据
  std::atomic<double> pos_;
  // 需要propagate实例中， 通过软件代码计算出速度填入数据
  std::atomic<double> vel_;
  // 计算速度的辅助变量, 保存前一次更新的时间
  std::chrono::high_resolution_clock::time_point previous_time_;
  // parse辅助变量
  const LegType& leg_;
  const JntType& jnt_;

  JointState(const LegType& leg, const JntType& jnt, double pos = 0, double vel = 0);
  virtual bool updateFrom(const class Feedback*) override;
};

struct JointCommand : public HwCommand {

  std::atomic<double>     command_;
  std::atomic<JntCmdType> mode_;

  // update辅助变量
  const LegType& leg_;
  const JntType& jnt_;

  JointCommand(const LegType& leg, const JntType& jnt, double cmd = 0, JntCmdType mode = JntCmdType::POS);
  virtual bool parseTo(class Command*) override;

};

class Joint : public HwUnit {
public:
  typedef JointCommand                    CmdType;
  typedef boost::shared_ptr<JointCommand> CmdTypeSp;
  typedef JointState                      StateType;
  typedef boost::shared_ptr<JointState>   StateTypeSp;

public:
  Joint(const std::string& name = "joint");

  virtual bool init(TiXmlElement*)   override;
  virtual HwStateSp getStataHandle() override;
  virtual HwCmdSp   getCmdHandle()   override;
  virtual HwStateSp getState()       override;
  virtual HwCmdSp   getCommand()     override;
  virtual void setState(const HwState&)     override;
  virtual void setCommand(const HwCommand&) override;

  virtual void publish() override;

  // for debug
  virtual void check() override;

protected:
  LegType leg_;
  JntType jnt_;
  StateTypeSp joint_state_;
  CmdTypeSp   joint_command_;

  static sensor_msgs::JointState s_joint_states_msg_;
  static ros::Publisher          s_joint_states_pub_;
  static bool                    s_ros_pub_init_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_ */
