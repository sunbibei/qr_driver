/*
 * mii_robot.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#include <system/robot/mii_robot.h>
#include <system/utils/cfg_reader.h>
#include "system/utils/log.h"
#include "middleware/hardware/hw_manager.h"
#include "middleware/hardware/joint.h"
#include "middleware/hardware/touchdown.h"

namespace middleware {

#define JOINT_TAG_NAME     ("joints")
#define TOUCHDOWN_TAG_NAME ("touchdowns")


void MiiRobot::auto_inst(const MiiString& __l, const MiiString& __t) {
  // MiiCfgReader::instance()->get_value(__l, "auto_inst");

}

MiiRobot::MiiRobot(const MiiString& l)
: Label(l), hw_manager_(HwManager::instance()) {
  ;
}

MiiRobot::~MiiRobot() {
  ;
}

bool MiiRobot::init() {
  auto cfg = MiiCfgReader::instance();



  HwManager::instance()->init();

  auto robot_cfg = cfg->find_first_item(getLabel());

  std::vector<std::string> names;
  robot_cfg.get_value(JOINT_TAG_NAME, names);
  joint_list_.reserve(names.size());

  std::stringstream ss;
  ss << "The joint in the Robot contains ";
  for (const auto& n : names) {
    Joint* j = getHardwareByName<Joint>(n);
    if (nullptr == j) {
      LOG_ERROR << "Something is wrong! Don't got the hardware " << n;
    } else {
      joint_list_.push_back(j);
      joint_list_by_name_.insert(std::make_pair(n, j));
    }

    ss << n << " ";
  }
  LOG_INFO << ss.str();

  names.clear();
  robot_cfg.get_value(TOUCHDOWN_TAG_NAME, names);
  td_list_.reserve(names.size());

  ss.str("");
  ss << "The touchdown in the Robot contains ";
  for (const auto& n : names) {
    TouchDown* td = getHardwareByName<TouchDown>(n);
    if (nullptr == td) {
      LOG_ERROR << "Something is wrong! Don't got the hardware " << n;
    } else {
      td_list_.push_back(td);
      td_list_by_name_.insert(std::make_pair(n, td));
    }

    ss << n << " ";
  }
  LOG_INFO << ss.str();

  return true;
}

void MiiRobot::addCommand(const std::string& name, double command) {
  if (joint_list_by_name_.end() == joint_list_by_name_.find(name)) {
    LOG_ERROR << "No named joint: " << name;
    return;
  }

  joint_list_by_name_[name]->updateJointCommand(command);
}
void MiiRobot::addCommand(const std::vector<std::string>& names, const std::vector<double>& commands) {
  if (names.size() != commands.size()) {
    LOG_ERROR << "No match size between names and commands("
        << names.size() << " v.s. " << commands.size() << ")";
    return;
  }
  for (size_t i = 0; i < names.size(); ++i)
    addCommand(names[i], commands[i]);
}

void MiiRobot::getJointNames(std::vector<std::string>& ret) {
  ret.clear();
  for (const auto& j : joint_list_by_name_)
    ret.push_back(j.first);
}

void MiiRobot::getJointPositions(std::vector<double>& ret) {
  ret.clear();
  for (const auto& j : joint_list_by_name_)
    ret.push_back(j.second->joint_position());
}

void MiiRobot::getJointVelocities(std::vector<double>& ret) {
  ret.clear();
  for (const auto& j : joint_list_by_name_)
    ret.push_back(j.second->joint_velocity());
}

void MiiRobot::getJointTorques(std::vector<double>& ret) {
  ret.clear();
  for (const auto& j : joint_list_by_name_)
    ret.push_back(j.second->joint_torque());
}

} /* namespace middleware */
