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

#include "system/utils/auto_instanceor.h"

namespace middleware {

#define JOINT_TAG_NAME     ("joints")
#define TOUCHDOWN_TAG_NAME ("touchdowns")


void MiiRobot::auto_inst(MiiStringConstRef __p, MiiStringConstRef __type) {
  AutoInstanceor::instance()->make_instance(__p, __type);
}

MiiRobot::MiiRobot(MiiStringConstRef __tag)
: prefix_tag_(Label::make_label(__tag, "robot")), hw_manager_(nullptr) {
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

  std::vector<std::string> names;
  cfg->get_value_fatal(Label::make_label(prefix_tag_, JOINT_TAG_NAME),
      "names", names);

  joint_list_.reserve(names.size());
  bool is_init_list_by_type = false;
  if (0 == (names.size() % JntType::N_JNTS)) {
    joint_list_by_type_.resize(names.size() / JntType::N_JNTS);
    for (auto& leg : joint_list_by_type_) {
      leg.resize(JntType::N_JNTS);
    }
    is_init_list_by_type = true;
  } else {
    LOG_WARNING << "The number of joint each leg is not " << JntType::N_JNTS
        << ", Can't initialize joint_list_by_type_";
  }

  std::stringstream ss;
  ss << "The joint in the Robot contains ";
  for (const auto& n : names) {
    Joint* j = Label::getHardwareByName<Joint>(n);
    if (nullptr == j) {
      LOG_ERROR << "Something is wrong! Don't got the hardware " << n;
    } else {
      joint_list_.push_back(j);
      joint_list_by_name_.insert(std::make_pair(n, j));
      if (is_init_list_by_type)
        joint_list_by_type_[j->owner_type()][j->joint_type()] = j;
    }

    ss << n << " ";
  }
  LOG_INFO << ss.str();

  names.clear();
  cfg->get_value_fatal(Label::make_label(prefix_tag_, TOUCHDOWN_TAG_NAME),
        "names", names);

  td_list_.reserve(names.size());
  if (is_init_list_by_type
      && (names.size() == joint_list_by_type_.size())) {
    td_list_by_type_.resize(names.size());
  } else {
    is_init_list_by_type = false;
  }

  ss.str("");
  ss << "The touchdown in the Robot contains ";
  for (const auto& n : names) {
    TouchDown* td = Label::getHardwareByName<TouchDown>(n);
    if (nullptr == td) {
      LOG_ERROR << "Something is wrong! Don't got the hardware " << n;
    } else {
      td_list_.push_back(td);
      td_list_by_name_.insert(std::make_pair(n, td));
      if (is_init_list_by_type)
        td_list_by_type_[td->leg_type()] = td;
    }

    ss << n << " ";
  }
  LOG_INFO << ss.str();

  hw_manager_ = HwManager::instance();
  hw_manager_->init();
  hw_manager_->run();
  return true;
}


bool MiiRobot::start() {
  return false;
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

void MiiRobot::addCommand(LegType _owner, JntType _jnt, double _command) {
  joint_manager_;
}

void MiiRobot::addCommand(const std::vector<LegType>&, const std::vector<JntType>&,
    const std::vector<double>&) {
  ;
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
