/*
 * ros_robothw.h
 *
 *  Created on: Jan 16, 2017
 *      Author: silence
 */

#ifndef INCLUDE_APPS_ROS_ROBOTHW_H_
#define INCLUDE_APPS_ROS_ROBOTHW_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <repository/resource/joint_manager.h>

#include "system/foundation/utf.h"

using namespace middleware;

class RosRobotHW: public hardware_interface::RobotHW {
public:
  virtual ~RosRobotHW();
  RosRobotHW(ros::NodeHandle&);

  /// \brief Initialize the hardware interface
  virtual void init();
  /// \brief Read the state from the robot hardware.
  virtual void read();
  /// \brief write the command to the robot hardware.
  virtual void write();

  /**
   * Perform (in realtime) all necessary hardware interface switches in order to start and stop the given controllers.
   * Start and stop list are disjoint. The feasability was checked in prepareSwitch() beforehand.
   */
  void doSwitch(const std::list<hardware_interface::ControllerInfo>&,
      const std::list<hardware_interface::ControllerInfo>&) override;

protected:
  bool canSwitch(
        const std::list<hardware_interface::ControllerInfo>&,
        const std::list<hardware_interface::ControllerInfo>&) const;

protected:
  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;
  // Joint API
  JointManager* jnt_manager_;
  // Interfaces
  hardware_interface::JointStateInterface        jnt_state_iface_;
  hardware_interface::PositionJointInterface     jnt_pos_iface_;
  hardware_interface::VelocityJointInterface     jnt_vel_iface_;
  hardware_interface::EffortJointInterface       jnt_tor_iface_; // Effort 接口实际并未实现
  hardware_interface::ForceTorqueSensorInterface td_state_iface_;
  hardware_interface::ImuSensorInterface         imu_state_iface_;


  bool velocity_interface_running_;
  bool position_interface_running_;
  bool effort_interface_running_;

  MiiVector<std::string> joint_names_;
  size_t num_joints_;

  MiiVector<const double*> jnt_pos_;
  MiiVector<const double*> jnt_vel_;
  MiiVector<const double*> jnt_tor_;

  // Shared memory
  MiiMap<MiiString, double*> jnt_pos_cmds_;
  MiiMap<MiiString, double*> jnt_vel_cmds_;
  MiiMap<MiiString, double*> jnt_tor_cmds_;

};

#endif /* INCLUDE_APPS_ROS_ROBOTHW_H_ */
