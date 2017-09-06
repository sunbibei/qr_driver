/*
 * test_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#include <apps/ros_wrapper.h>
#include <iostream>

int main(int argc, char* argv[]) {
  bool use_sim_time = false;

  ros::init(argc, argv, "dragon_driver");
  ros::NodeHandle nh;
  if (ros::param::get("use_sim_time", use_sim_time)) {
    LOG_INFO << ("use_sim_time is set!!");
  }

  middleware::RosWrapper* interface = middleware::RosWrapper::instance();
  if (nullptr == interface) {
    LOG_FATAL << "Can't get the instance of QrRosWrapper!";
    return -1;
  }
  interface->start();

  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::waitForShutdown();

  interface->halt();

  LOG_INFO << "dragon_driver shutdown... ...";
  return 0;
}
