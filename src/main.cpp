/*
 * test_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#include <apps/ros_wrapper.h>
#include <iostream>

int main(int argc, char* argv[]) {
  google::InitGoogleLogging("qr_driver");
  google::FlushLogFiles(google::GLOG_INFO);

  ros::init(argc, argv, "qr_driver");
  ros::NodeHandle nh("~");

  if (nullptr == RosWrapper::create_instance())
    LOG_FATAL << "Can't get the instance of RosWrapper!";
  RosWrapper::instance()->start();

  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::waitForShutdown();

  LOG_INFO << "qr_driver shutdown... ...";
  RosWrapper::destroy_instance();
  return 0;
}
