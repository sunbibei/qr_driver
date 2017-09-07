/*
 * mii_robot.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#include <system/resources/joint.h>
#include <system/foundation/label.h>
#include <system/resources/touchdown.h>
#include <system/platform/hw_manager.h>
#include <system/resources/joint_manager.h>
#include <system/platform/propagate/propagate_manager.h>
#include "system/robot/mii_robot.h"
#include "system/foundation/cfg_reader.h"
#include "system/platform/thread/threadpool.h"
#include "system/foundation/auto_instanceor.h"

namespace middleware {

#define JOINT_TAG_NAME     ("joints")
#define TOUCHDOWN_TAG_NAME ("touchdowns")


void MiiRobot::auto_inst(const MiiString& __p, const MiiString& __type) {
  if (AutoInstanceor::instance()->make_instance(__p, __type)) {
    LOG_INFO << "Create instance(" << __type << " " << __p << ")";
  } else {
    LOG_WARNING << "Create instance(" << __type << " " << __p << ") fail!";
  }
}

MiiRobot::MiiRobot(const MiiString& __tag)
: prefix_tag_(Label::make_label(__tag, "robot")),
  hw_manager_(nullptr), jnt_manager_(nullptr) {
  ;
}

MiiRobot::~MiiRobot() {
  ;
}

/**
 * This method creates the part of singleton.
 */
void MiiRobot::create_system_instance() {
  if (nullptr == ThreadPool::create_instance())
    LOG_WARNING << "Create the singleton 'ThreadPool' has failed.";

  if (nullptr == JointManager::create_instance())
    LOG_WARNING << "Create the singleton 'JointManager' has failed.";

  if (nullptr == HwManager::create_instance())
    LOG_WARNING << "Create the singleton 'HwManager' has failed.";

  // The PropagateManager does not instance.
  // if (nullptr == PropagateManager::create_instance())
  //   LOG_WARNING << "Create the singleton 'HwManager' has failed.";
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
  LOG_WARNING << "Auto instance has finished. The results list as follow:";
  Label::printfEveryInstance();

  hw_manager_  = HwManager::instance();
  hw_manager_->init();

  jnt_manager_ = JointManager::instance();
  return true;
}


bool MiiRobot::start() {
  hw_manager_->run();
  return false;
}


inline void MiiRobot::addCommand(const MiiString& name, double command) {
  jnt_manager_->addJointCommand(name, command);
}

inline void MiiRobot::addCommand(const MiiVector<MiiString>& names,
    const MiiVector<double>& commands) {
  // jnt_manager_->addJointCommand(names, commands);
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
    const MiiVector<double>&) {

}

void MiiRobot::getJointNames(MiiVector<MiiString>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_name());
  }
}

void MiiRobot::getJointPositions(MiiVector<double>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_position());
  }
}

void MiiRobot::getJointVelocities(MiiVector<double>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_velocity());
  }
}

void MiiRobot::getJointTorques(MiiVector<double>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_torque());
  }
}

} /* namespace middleware */
