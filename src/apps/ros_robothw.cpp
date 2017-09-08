/*
 * ros_robothw.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: silence
 */

#include <apps/ros_robothw.h>

RosRobotHW::~RosRobotHW() {
  for (auto& cmd : jnt_pos_cmds_) {
    delete cmd.second;
    cmd.second = nullptr;
  }

  for (auto& cmd : jnt_vel_cmds_) {
    delete cmd.second;
    cmd.second = nullptr;
  }

  for (auto& cmd : jnt_tor_cmds_) {
    delete cmd.second;
    cmd.second = nullptr;
  }
}

RosRobotHW::RosRobotHW(ros::NodeHandle& nh)
  : nh_(nh), jnt_manager_(nullptr) {

  jnt_manager_ = JointManager::instance();
  // robot->getJointNames(joint_names_);
  num_joints_ = joint_names_.size();

  init();
  LOG_INFO << "Loaded RosRobotHW";
}

/// \brief Initialize the hardware interface
void RosRobotHW::init() {
  if (jnt_manager_->empty()) {
    LOG_FATAL <<
        "No joints found on parameter server for controller, "
        << "did you load the proper yaml file?";
  }

  MiiString jnt_name;
  const double *_pos, *_vel, *_tor;
  double* _cmd = nullptr;
  // Initialize controller
  for (size_t i = 0; i < jnt_manager_->size(); ++i) {
    jnt_name = (*jnt_manager_)[i]->joint_name();
    jnt_manager_->joint_position_const_pointer(jnt_name, _pos);
    jnt_manager_->joint_velocity_const_pointer(jnt_name, _vel);
    jnt_manager_->joint_torque_const_pointer(jnt_name,   _tor);
    // Create joint state interface
    jnt_state_iface_.registerHandle(
        hardware_interface::JointStateHandle(jnt_name,
            _pos, _vel, _tor));

    // Create position joint interface
    _cmd = new double;
    jnt_pos_iface_.registerHandle(
        hardware_interface::JointHandle(
            jnt_state_iface_.getHandle(jnt_name),
            _cmd));
    jnt_pos_cmds_.insert(std::make_pair(jnt_name, _cmd));

    // Create velocity joint interface
    _cmd = new double;
    jnt_vel_iface_.registerHandle(
        hardware_interface::JointHandle(
            jnt_state_iface_.getHandle(jnt_name),
            _cmd));
    jnt_vel_cmds_.insert(std::make_pair(jnt_name, _cmd));

    // Create effort joint interface
    _cmd = new double;
    jnt_tor_iface_.registerHandle(
        hardware_interface::JointHandle(
            jnt_state_iface_.getHandle(jnt_name),
            _cmd));
    jnt_tor_cmds_.insert(std::make_pair(jnt_name, _cmd));
  } // end for (size_t i = 0; i < jnt_manager_->size(); ++i)

  registerInterface(&jnt_state_iface_); // From RobotHW base class.
  registerInterface(&jnt_pos_iface_);   // From RobotHW base class.
  registerInterface(&jnt_vel_iface_);   // From RobotHW base class.
  registerInterface(&jnt_tor_iface_);   // From RobotHW base class.
  velocity_interface_running_ = false;
  position_interface_running_ = false;
  effort_interface_running_   = false;
}

/// \brief Read the state from the robot hardware.
void RosRobotHW::read() {
  // auto read from Joint
}

/// \brief write the command to the robot hardware.
void RosRobotHW::write() {
  // TODO
  if (velocity_interface_running_) {
    for (auto& jnt : *jnt_manager_)
      jnt->updateJointCommand(*jnt_pos_cmds_[jnt->joint_name()]);
  } else if (position_interface_running_) {
    for (auto& jnt : *jnt_manager_)
      jnt->updateJointCommand(*jnt_tor_cmds_[jnt->joint_name()]);
  } else if (effort_interface_running_) {
    LOG_WARNING << "NO IMPLEMENTES"; // Nothing to do here
  } else {
    ; // Nothing to do here
  }
}

bool RosRobotHW::canSwitch(
      const std::list<hardware_interface::ControllerInfo> &start_list,
      const std::list<hardware_interface::ControllerInfo> &stop_list) const {
  for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
      start_list.begin(); controller_it != start_list.end();
      ++controller_it) {
    if (0 == controller_it->type.compare(
        "hardware_interface::VelocityJointInterface")) {
      if (velocity_interface_running_) {
        LOG_ERROR << controller_it->name.c_str()
            << ": An interface of that type ("
            << controller_it->type.c_str()
            << ") is already running";
        return false;
      }
      if (position_interface_running_) {
        bool error = true;
        for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
            stop_list.begin();
            stop_controller_it != stop_list.end();
            ++stop_controller_it) {
          if (0 == stop_controller_it->type.compare(
              "hardware_interface::PositionJointInterface")) {
            error = false;
            break;
          }
        }
        if (error) {
          LOG_ERROR << controller_it->name.c_str()
              << " (type " << controller_it->type.c_str()
              << ") can not be run simultaneously with a PositionJointInterface";
          return false;
        }
      }
      if (effort_interface_running_) {
        bool error = true;
        for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
            stop_list.begin();
            stop_controller_it != stop_list.end();
            ++stop_controller_it) {
          if (0 == stop_controller_it->type.compare(
              "hardware_interface::EffortJointInterface")) {
            error = false;
            break;
          }
        }
        if (error) {
          LOG_ERROR << controller_it->name.c_str()
              << " (type " << controller_it->type.c_str()
              << ") can not be run simultaneously with a EffortJointInterface";
          return false;
        }
      }
    } else if (0 == controller_it->type.compare(
        "hardware_interface::PositionJointInterface")) {
      if (position_interface_running_) {
        LOG_ERROR << "%s: An interface of that type (%s) is already running"
            << controller_it->name.c_str()
            << controller_it->type.c_str();
        return false;
      }
      if (velocity_interface_running_) {
        bool error = true;
        for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
            stop_list.begin();
            stop_controller_it != stop_list.end();
            ++stop_controller_it) {
          if (0 == stop_controller_it->type.compare(
              "hardware_interface::VelocityJointInterface")) {
            error = false;
            break;
          }
        }
        if (error) {
          LOG_ERROR << "%s (type %s) can not be run simultaneously with a VelocityJointInterface"
              << controller_it->name.c_str()
              << controller_it->type.c_str();
          return false;
        }
      }
      if (effort_interface_running_) {
          bool error = true;
          for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
              stop_list.begin();
              stop_controller_it != stop_list.end();
              ++stop_controller_it) {
            if (0 == stop_controller_it->type.compare(
                "hardware_interface::EffortJointInterface")) {
              error = false;
              break;
            }
          }
          if (error) {
            LOG_ERROR << controller_it->name.c_str() << " (type "
                << controller_it->type.c_str()
                << ") can not be run simultaneously with a VelocityJointInterface";
            return false;
          }
        }
    } else if (0 == controller_it->type.compare(
        "hardware_interface::EffortJointInterface")) {
      if (effort_interface_running_) {
        LOG_ERROR << controller_it->name.c_str()
            << ": An interface of that type ("
            << controller_it->type.c_str()
            << ") is already running";
        return false;
      }
      if (velocity_interface_running_) {
        bool error = true;
        for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
            stop_list.begin();
            stop_controller_it != stop_list.end();
            ++stop_controller_it) {
          if (0 == stop_controller_it->type.compare(
              "hardware_interface::VelocityJointInterface")) {
            error = false;
            break;
          }
        }
        if (error) {
          LOG_ERROR << controller_it->name.c_str()
                    << " (type " << controller_it->type.c_str()
                    << ") can not be run simultaneously with a VelocityJointInterface";
          return false;
        }
      }
      if (position_interface_running_) {
          bool error = true;
          for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
              stop_list.begin();
              stop_controller_it != stop_list.end();
              ++stop_controller_it) {
            if (0 == stop_controller_it->type.compare(
                "hardware_interface::PositionJointInterface")) {
              error = false;
              break;
            }
          }
          if (error) {
            LOG_ERROR << controller_it->name.c_str()
                << " (type " << controller_it->type.c_str()
                << ") can not be run simultaneously with a PositionJointInterface";
            return false;
          }
        }
    }
  }

  // we can always stop a controller
  return true;
}

void RosRobotHW::doSwitch(const std::list<hardware_interface::ControllerInfo>&start_list,
    const std::list<hardware_interface::ControllerInfo>&stop_list) {
  for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
      stop_list.begin(); controller_it != stop_list.end();
      ++controller_it) {
    if (0 == controller_it->type.compare(
        "hardware_interface::VelocityJointInterface")) {
      velocity_interface_running_ = false;
      LOG_INFO << ("Stopping velocity interface");
    }
    if (0 == controller_it->type.compare(
        "hardware_interface::PositionJointInterface")) {
      position_interface_running_ = false;
      // std::vector<double> tmp;
      // robot_->closeServo(tmp);
      LOG_INFO << ("Stopping position interface");
    }
    if (0 == controller_it->type.compare(
        "hardware_interface::EffortJointInterface")) {
      effort_interface_running_ = false;
      // std::vector<double> tmp;
      // robot_->closeServo(tmp);
      LOG_INFO << ("Stopping position interface");
    }
  }
  for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
      start_list.begin(); controller_it != start_list.end();
      ++controller_it) {
    if (0 == controller_it->type.compare(
        "hardware_interface::VelocityJointInterface")) {
      velocity_interface_running_ = true;
      LOG_INFO << ("Starting velocity interface");
    }
    if (0 == controller_it->type.compare(
        "hardware_interface::PositionJointInterface")) {
      position_interface_running_ = true;
      // robot_->uploadProg();
      LOG_INFO << ("Starting position interface");
    }
    if (0 == controller_it->type.compare(
        "hardware_interface::EffortJointInterface")) {
      effort_interface_running_ = true;
      // robot_->uploadProg();
      LOG_INFO << ("Starting effort interface");
    }
  }
}
