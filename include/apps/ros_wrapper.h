/*
 * qr_ros_wrapper.h
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#ifndef INCLUDE_QR_ROS_WRAPPER_H_
#define INCLUDE_QR_ROS_WRAPPER_H_

#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <controller_manager/controller_manager.h>

#include "system/robot/mii_robot.h"

#define DEBUG_TOPIC
#ifdef DEBUG_TOPIC
#include <repository/resource/joint.h>
#endif

using namespace middleware;

class RosWrapper : public MiiRobot {
SINGLETON_DECLARE(RosWrapper)

public:
  virtual void create_system_instance() override;
  virtual bool start() override;
  void halt();

private:
  // 发布实时消息， 例如"/joint_states"
  void publishRTMsg();
  void rosControlLoop();
  // 测试消息回调函数
#ifdef DEBUG_TOPIC
  void cbForDebug(const std_msgs::Int32ConstPtr&);
  ros::Subscriber cmd_sub_;
#endif

private:
  // The ROS handle
  ros::NodeHandle nh_;

  bool alive_;
  // About ROS control
  std::chrono::milliseconds rt_duration_; // 实时消息发布频率， 默认是50Hz(使用周期表示, 即20ms）
  std::chrono::milliseconds ros_ctrl_duration_; // ros_control_thread_循环频率， 默认是100Hz(使用周期表示, 即10ms）
  bool use_ros_control_;
  boost::shared_ptr<class RosRobotHW> hardware_interface_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

#endif /* INCLUDE_QR_ROS_WRAPPER_H_ */
