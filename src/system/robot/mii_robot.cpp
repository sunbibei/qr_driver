/*
 * mii_robot.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#include <system/robot/mii_robot.h>
#include <system/utils/cfg_reader.h>
#include <system/utils/utf.h>
#include "middleware/hardware/hw_manager.h"
#include "middleware/hardware/joint.h"
#include "middleware/hardware/joint_manager.h"
#include "middleware/hardware/touchdown.h"

#include "system/utils/auto_instanceor.h"

namespace middleware {

#define JOINT_TAG_NAME     ("joints")
#define TOUCHDOWN_TAG_NAME ("touchdowns")


void MiiRobot::auto_inst(ConstRef<MiiString> __p, ConstRef<MiiString> __type) {
  AutoInstanceor::instance()->make_instance(__p, __type);
}

MiiRobot::MiiRobot(ConstRef<MiiString> __tag)
: prefix_tag_(Label::make_label(__tag, "robot")),
  hw_manager_(nullptr), jnt_manager_(nullptr) {
  ;
}

MiiRobot::~MiiRobot() {
  ;
}

bool MiiRobot::init() {
  create_system_instance();

  auto cfg = MiiCfgReader::instance();
  if (nullptr == cfg) {
    LOG_FATAL << "The MiiCfgReader::create_instance(MiiStringConstRef) "
        << "method must to be called by subclass before MiiRobot::init()";
  }

  // All of the objects mark with "auto_inst" in the configure file
  // will be instanced here.
  cfg->registerCallbackAndExcute("auto_inst", MiiRobot::auto_inst);
  // Just for debug
  Label::printfEveryInstance();

  jnt_manager_ = JointManager::instance();

  hw_manager_ = HwManager::instance();
  hw_manager_->init();
  hw_manager_->run();
  return true;
}


bool MiiRobot::start() {
  return false;
}


inline void MiiRobot::addCommand(ConstRef<MiiString> name, double command) {
  jnt_manager_->addJointCommand(name, command);
}

inline void MiiRobot::addCommand(ConstRef<MiiVector<MiiString>> names, ConstRef<MiiVector<double>> commands) {
  jnt_manager_->addJointCommand(names, commands);
  /*if (names.size() != commands.size()) {
    LOG_ERROR << "No match size between names and commands("
        << names.size() << " v.s. " << commands.size() << ")";
    return;
  }
  for (size_t i = 0; i < names.size(); ++i)
    addCommand(names[i], commands[i]);*/
}

void MiiRobot::addCommand(LegType _owner, JntType _jnt, double _command) {
  jnt_manager_->addJointCommand(_owner, _jnt, _command);
}

void MiiRobot::addCommand(const MiiVector<LegType>&, const MiiVector<JntType>&,
    const std::vector<double>&) {
  jnt_manager_->addJointCommand();
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
