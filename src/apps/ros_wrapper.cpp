/*
 * qr_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#include <apps/ros_robothw.h>
#include <apps/ros_wrapper.h>
#include <repository/resource/joint_manager.h>
#include <repository/resource/imu_sensor.h>
#include <repository/resource/force_sensor.h>
#include "foundation/auto_instanceor.h"
#include "foundation/cfg_reader.h"
#include "system/platform/thread/threadpool.h"

#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#define ROS_CTRL_THREAD ("ros_control")
#define RT_PUB_THREAD   ("rt_publisher")

SINGLETON_IMPL_NO_CREATE(RosWrapper)

RosWrapper* RosWrapper::create_instance(const MiiString& __tag) {
  if (nullptr != instance_) {
    LOG_WARNING << "This method 'create_instance()' is called twice.";
  } else {
    instance_ = new RosWrapper(__tag);
  }
  return instance_;
}

RosWrapper::RosWrapper(const MiiString& __tag)
  : MiiRobot(Label::make_label(__tag, "robot")), root_tag_(__tag), alive_(false),
    rt_duration_(1000/50), ros_ctrl_duration_(1000/100), use_ros_control_(false) {
  // LOG_DEBUG << "Enter the roswrapper construction";
  // google::InitGoogleLogging("qr_driver");
  // google::SetLogDestination(google::GLOG_INFO, "/path/to/log/INFO_");
  FLAGS_colorlogtostderr = true;
  // google::FlushLogFiles(google::GLOG_INFO);
  // LOG_DEBUG << "Leave the roswrapper construction";
  ; // Nothing to do here, all of variables initialize in the method @start()
}

RosWrapper::~RosWrapper() {
  // LOG_DEBUG << "Enter the roswrapper deconstruction";
  halt();
  // AutoInstanceor::destroy_instance();
  MiiCfgReader::destroy_instance();
  // LOG_DEBUG << "Leave the roswrapper deconstruction";
  // google::ShutdownGoogleLogging();
}

void RosWrapper::create_system_instance() {

  MiiString str;
  // if (nh_.getParam("configure", cfg)) {
  if (!ros::param::get("~configure", str)) {
    LOG_FATAL << "RosWapper can't find the 'configure' parameter "
        << "in the parameter server. Did you forget define this parameter.";
  }
  if (nullptr == MiiCfgReader::create_instance(str))
    LOG_FATAL << "Create the singleton 'MiiCfgReader' has failed.";

  if (!ros::param::get("~library", str)) {
    LOG_FATAL << "RosWapper can't find the 'configure' parameter "
        << "in the parameter server. Did you forget define this parameter.";
  }
  LOG_DEBUG << str;
  if (nullptr == AutoInstanceor::create_instance(str))
    LOG_FATAL << "Create the singleton 'AutoInstanceor' has failed.";

  // LOG_DEBUG << "==========RosWrapper::create_system_instance==========>>";
  MiiRobot::create_system_instance();
}

bool RosWrapper::start() {
  if (!init()) LOG_FATAL << "Robot initializes fail!";

  LOG_INFO << "MiiRobot initialization has completed.";
  bool debug = false;
  ros::param::get("~debug", debug);
  google::SetStderrLogging(debug ?
      google::GLOG_INFO : google::GLOG_WARNING);

  ros::param::get("~use_ros_control", use_ros_control_);
  if (use_ros_control_) {
    hardware_interface_.reset(
              new RosRobotHW(nh_, Label::make_label(root_tag_, "roswrapper")));
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
  }
  
  double frequency = 50.0;
  ros::param::get("~rt_frequency", frequency);
  if (frequency > 0)
    rt_duration_ = std::chrono::milliseconds((int)(1000.0 / frequency));

  ThreadPool::instance()->add(RT_PUB_THREAD, &RosWrapper::publishRTMsg, this);

  // For debug
#ifdef DEBUG_TOPIC
  cmd_sub_ = nh_.subscribe<std_msgs::Int32>("debug", 100,
      &RosWrapper::cbForDebug, this);
#endif
  bool ret = MiiRobot::start();
  // LOG_DEBUG << "==========RosWrapper::start==========>>";
  return ret;
}

inline void __fill_jnt_data(sensor_msgs::JointState& to, JointManager* from) {
  to.position.clear();
  to.velocity.clear();
  to.effort.clear();
  for (const auto& jnt : *from) {
    to.position.push_back(jnt->joint_position());
    to.velocity.push_back(jnt->joint_velocity());
    to.effort.push_back(jnt->joint_torque());
  }
  to.header.stamp = ros::Time::now();
}

inline void __fill_imu_data(sensor_msgs::Imu& to, ImuSensor* from) {
  to.orientation.x = from->orientation_const_pointer()[0];
  to.orientation.y = from->orientation_const_pointer()[1];
  to.orientation.z = from->orientation_const_pointer()[2];
  to.orientation.w = from->orientation_const_pointer()[3];
  for (size_t i = 0; i < to.orientation_covariance.size(); ++i)
    to.orientation_covariance[i] = from->orientation_covariance_const_pointer()[i];

  to.angular_velocity.x = from->angular_velocity_const_pointer()[0];
  to.angular_velocity.y = from->angular_velocity_const_pointer()[1];
  to.angular_velocity.z = from->angular_velocity_const_pointer()[2];
  for (size_t i = 0; i < to.angular_velocity_covariance.size(); ++i)
    to.angular_velocity_covariance[i] = from->angular_velocity_covariance_const_pointer()[i];

  to.linear_acceleration.x = from->linear_acceleration_const_pointer()[0];
  to.linear_acceleration.y = from->linear_acceleration_const_pointer()[1];
  to.linear_acceleration.z = from->linear_acceleration_const_pointer()[2];
  for (size_t i = 0; i < to.linear_acceleration_covariance.size(); ++i)
    to.linear_acceleration_covariance[i] = from->linear_acceleration_covariance_const_pointer()[i];

  to.header.stamp = ros::Time::now();
}

inline void __fill_force_data(std_msgs::Int32MultiArray& to, MiiVector<ForceSensor*>& from) {
  for (const LegType& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    // LOG_DEBUG << "enter: " << leg << " " << to.data.size() << " " << from.size();// << " " << from[leg];
    to.data[leg] = from[leg]->force_data();
  }
}

void RosWrapper::publishRTMsg() {
  ros::Publisher jnt_puber
      = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher imu_puber
      = nh_.advertise<sensor_msgs::Imu>("imu", 1);
  ros::Publisher force_puber
      = nh_.advertise<std_msgs::Int32MultiArray>("foot_forces", 1);
  sensor_msgs::JointState   __jnt_msg;
  sensor_msgs::Imu          __imu_msg;
  std_msgs::Int32MultiArray __f_msg;
  getJointNames(__jnt_msg.name);

  __imu_msg.header.frame_id = "imu";

  __f_msg.data.resize(4);
  __f_msg.layout.data_offset = 0;
  for (const MiiString& l : {"fl", "fr", "hl", "hr"}) {
    std_msgs::MultiArrayDimension dim;
    dim.label = "foot_" + l;
    dim.size  = 1;
    dim.stride= 1;
    __f_msg.layout.dim.push_back(dim);
  }

  TIMER_INIT
  while (alive_ && ros::ok()) {
    if (jnt_puber.getNumSubscribers()) {
      __fill_jnt_data(__jnt_msg, jnt_manager_);
      jnt_puber.publish(__jnt_msg);
    }
    if (imu_puber.getNumSubscribers()) {
      __fill_imu_data(__imu_msg, imu_sensor_);
      imu_puber.publish(__imu_msg);
    }

    if (force_puber.getNumSubscribers()) {
      __fill_force_data(__f_msg, td_list_by_type_);
      force_puber.publish(__f_msg);
    }

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

  controller_manager_.reset();
  hardware_interface_.reset();
}

#ifdef DEBUG_TOPIC
void RosWrapper::cbForDebug(const std_msgs::Int32ConstPtr& msg) {
  for (auto& jnt : *jnt_manager_) {
    // LOG_DEBUG << "Joint " << jnt->joint_name() << " adds the command " << msg->data;
    jnt->updateJointCommand(msg->data);
  }
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
