#include "qr_driver_test.h"

namespace qr_driver_test {

using namespace hardware_interface;

QrDriverTestController::QrDriverTestController() :
  is_control_(false) {
}

QrDriverTestController::~QrDriverTestController() {

}

/**************************************************************************
   Description: initialize joints from robot_description
**************************************************************************/
bool QrDriverTestController::init(RobotHW* robot, ros::NodeHandle &n) {
    int count = 0;
    PositionJointInterface* iface = robot->get<PositionJointInterface>();
    while(true) {
        std::string joint_name;
        std::string param_name = std::string("joint_" + std::to_string(count));
        if (ros::param::get(param_name.c_str(), joint_name)) {
            std::cout << "Get Joint Name: " << joint_name << std::endl;
            joint_handles_.push_back(iface->getHandle(joint_name));
        } else {
            break;
        }
        ++count;
    }

    count = 0;
    ForceTorqueSensorInterface* force = robot->get<ForceTorqueSensorInterface>();
    while(true) {
        std::string joint_name;
        std::string param_name = std::string("td_" + std::to_string(count));
        if (ros::param::get(param_name.c_str(), joint_name)) {
            std::cout << "Get Touchdown Name: " << joint_name << std::endl;
            td_handles_.push_back(force->getHandle(joint_name));
        } else {
            break;
        }
        ++count;
    }

    std::string str = "";
    if (ros::param::get("imu", str)) {
      ImuSensorInterface* imu = robot->get<ImuSensorInterface>();
      std::cout << "Get IMU Name: " << str << std::endl;
      imu_handle_ = imu->getHandle(str);
    }

    reset_sub_ = n.subscribe<std_msgs::Bool>("QrDriverTest", 1,
       &QrDriverTestController::cbForReset, this);
    return true;
}

void QrDriverTestController::cbForReset(const std_msgs::BoolConstPtr& msg) {
  is_control_ = msg->data;
}

/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: set model initial state
**************************************************************************/
void QrDriverTestController::starting(const ros::Time& time) {

}
/**************************************************************************
   Author: WangShanren
   Date: 2017.2.22
   Description: design state meachine: Adjust CoG <---> Switch Swing Leg
**************************************************************************/
void QrDriverTestController::update(const ros::Time&, const ros::Duration&) {
  if (!is_control_) return;
  is_control_ = false;

  std::cout << "JointStates: ";
  for (const auto& j : joint_handles_) {
    printf("%04f ", j.getPosition());
  }
  std::cout << std::endl;

  std::cout << "ForceSensor: ";

  for (const auto& f : td_handles_) {
    printf("%04f ", *(f.getForce()));
  }
  std::cout << std::endl;

  if (!imu_handle_.getName().empty()) {
    ; // Output the information of IMU
    std::cout << "ImuSensor:   " << std::endl;
    auto d = imu_handle_.getAngularVelocity();
    std::cout << d[0] << " " << d[1] << " " << d[2] << std::endl;
    d = imu_handle_.getLinearAcceleration();
    std::cout << d[0] << " " << d[1] << " " << d[2] << std::endl;
    d = imu_handle_.getOrientation();
    std::cout << d[0] << " " << d[1] << " " << d[2] << " " << d[3] << std::endl;
  }
}

} /* end namespace qr_driver_test */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(qr_driver_test::QrDriverTestController, controller_interface::ControllerBase)
