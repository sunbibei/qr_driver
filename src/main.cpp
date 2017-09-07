/*
 * test_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#include <apps/ros_wrapper.h>
#include <iostream>

using middleware::RosWrapper;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "qr_driver");
  ros::NodeHandle nh("~");

  if (nullptr == RosWrapper::instance())
    LOG_FATAL << "Can't get the instance of QrRosWrapper!";
  RosWrapper::instance()->start();

  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::waitForShutdown();

  LOG_INFO << "dragon_driver shutdown... ...";
  return 0;
}
