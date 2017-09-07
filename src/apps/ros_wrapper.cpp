/*
 * qr_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#include <apps/ros_robothw.h>
#include <apps/ros_wrapper.h>
#include "system/foundation/cfg_reader.h"
#include "system/platform/thread/threadpool.h"

namespace middleware {

#define ROS_CTRL_THREAD ("ros_control")
#define RT_PUB_THREAD   ("rt_publish")

RosWrapper* RosWrapper::instance_ = nullptr;

RosWrapper* RosWrapper::instance() {
  if (nullptr == instance_) {
    instance_ = new RosWrapper;
    LOG_INFO << "Create the singleton instance: RosWrapper";
  }

  LOG_INFO << "Return the singleton instance: RosWrapper";
  return instance_;
}

RosWrapper::RosWrapper()
  : MiiRobot("qr.wrapper"),
    alive_(false),
    rt_duration_(1000/50),
    ros_ctrl_duration_(1000/100),
    use_ros_control_(true) {
  google::InitGoogleLogging("dragon_ros_wrapper");
  // google::SetLogDestination(google::GLOG_INFO, "/path/to/log/INFO_");
  // google::LogMessage::Init();
  FLAGS_colorlogtostderr = true;
  google::FlushLogFiles(google::GLOG_INFO);
  ; // Nothing to do here, all of variables initialize in the method @start()
}

void RosWrapper::create_system_instance() {
  MiiString cfg;
  if (ros::param::get("~configure", cfg)) {
    LOG_FATAL << "RosWapper can't find the 'configure' parameter "
        << "in the parameter server. Did you forget define this parameter.";
  }
  if (MiiCfgReader::create_instance(cfg))
    LOG_FATAL << "Create the singleton 'MiiCfgReader' has failed.";

  return MiiRobot::create_system_instance();
}

bool RosWrapper::start() {
  bool debug = false;
  ros::param::get("~debug", debug);
  if (debug) google::SetStderrLogging(google::GLOG_INFO);
  else google::SetStderrLogging(google::GLOG_WARNING);

  ros::param::get("~use_ros_control", use_ros_control_);
  if (use_ros_control_) {
    // hardware_interface_.reset(
    //           new middleware::RosRobotHW(nh_, robot_));
    controller_manager_.reset(
        new controller_manager::ControllerManager(
            hardware_interface_.get(), nh_));
  }

  alive_ = true;
  if (use_ros_control_) {
    double frequency = 100.0;
    ros::param::get("~ctrl_loop_frequency", frequency);
    if (frequency > 0)
      ros_ctrl_duration_ = std::chrono::milliseconds((int)(1000.0 / frequency));

    ThreadPool::instance()->add(ROS_CTRL_THREAD, &RosWrapper::rosControlLoop, this);
  } else {
    double frequency = 50.0;
    ros::param::get("~rt_frequency", frequency);
    if (frequency > 0)
      rt_duration_ = std::chrono::milliseconds((int)(1000.0 / frequency));

    ThreadPool::instance()->add(RT_PUB_THREAD, &RosWrapper::publishRTMsg, this);
  }

  // For debug
#ifdef DEBUG_TOPIC
  cmd_sub_ = nh_.subscribe<std_msgs::Int32>("debug", 100,
      &RosWrapper::cbForDebug, this);
#endif
  return true;
}

void RosWrapper::publishRTMsg() {
  ros::Publisher joint_pub
    = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  sensor_msgs::JointState __msg;
  getJointNames(__msg.name);

  TIMER_INIT
  while (alive_ && ros::ok()) {
    getJointPositions(__msg.position);
    getJointVelocities(__msg.velocity);
    getJointTorques(__msg.effort);
    __msg.header.stamp = ros::Time::now();

    joint_pub.publish(__msg);
    TIMER_CONTROL(rt_duration_)
  }

  alive_ = false;
}

void RosWrapper::rosControlLoop() {
  ros::Duration elapsed_time;
  struct timespec last_time, current_time;
  static const double BILLION = 1000000000.0;
  clock_gettime(CLOCK_MONOTONIC, &last_time);

  TIMER_INIT
  while (alive_ && ros::ok()) {
    // Input
    hardware_interface_->read();
    // Control
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time = ros::Duration(current_time.tv_sec - last_time.tv_sec
        + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);

    controller_manager_->update(ros::Time::now(), elapsed_time);
    last_time = current_time;
    // Output
    hardware_interface_->write();

    TIMER_CONTROL(ros_ctrl_duration_)
  }

  alive_ = false;
}

void RosWrapper::halt() {
  alive_ = false;
  ThreadPool::instance()->stop(RT_PUB_THREAD);
  ThreadPool::instance()->stop(ROS_CTRL_THREAD);

  controller_manager_.reset();
  hardware_interface_.reset();
}

RosWrapper::~RosWrapper() {
  halt();
  if (nullptr != instance_) {
    delete instance_;
    instance_ = nullptr;
  }

  google::ShutdownGoogleLogging();
}

#ifdef DEBUG_TOPIC
void RosWrapper::cbForDebug(const std_msgs::Int32ConstPtr& msg) {
  // 实现方式0
  /*LOG_INFO << "test write style 0";
  for (auto& jnt : robot_->jnt_names_) {
    Motor::CmdType cmd(msg->data, Motor::CmdType::MODE_POS_);
    robot_->addCommand(jnt, cmd);
  }*/
  // 实现方式1
  /*LOG_INFO << "test write style 1";
  for (auto& jnt : robot_->jnt_names_) {
    Motor::CmdTypeSp cmd(new Motor::CmdType(msg->data, Motor::CmdType::MODE_POS_));
    robot_->addCommand(jnt, cmd);
  }*/
  // 实现方式2
  /*LOG_INFO << "test write style 2";
  std::vector<HwCmdSp> cmd_vec;
  std::vector<std::string> cmd_name;
  for (const auto& jnt : robot_->jnt_names_) {
    auto cmd = boost::dynamic_pointer_cast<Joint::CmdType>(robot_->hw_unit_[jnt]->getCommand());
    cmd->command_ = msg->data;
    cmd->mode_    = JntCmdType::POS;

    cmd_vec.push_back(cmd);
    cmd_name.push_back(jnt);
  }
  robot_->addCommand(cmd_name, cmd_vec);*/

  LOG_INFO << "Add Command Successful";
}
#endif

} /* namespace qr_driver */