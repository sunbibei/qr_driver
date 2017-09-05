/*
 * joint_manager.cpp
 *
 *  Created on: Sep 1, 2017
 *      Author: silence
 */

#include <middleware/hardware/joint_manager.h>

namespace middleware {

JointManager* JointManager::instance_ = nullptr;

JointManager* JointManager::create_instance() {
  if (nullptr != instance_)
    LOG_WARNING << "This method 'JointManager::create_instance()' is called twice.";
  else
    instance_ = new JointManager;

  return instance_;
}

JointManager* JointManager::instance() {
  if (nullptr == instance_)
    LOG_WARNING << "This method JointManager::instance() should be called after "
        << "JointManager::create_instance()";

  return instance_;
}

void JointManager::destroy_instance() {
  if (nullptr != instance_) {
    delete instance_;
    instance_ = nullptr;
  }
}

JointManager::JointManager() {
}

JointManager::~JointManager() {
  // Nothing to do here
}

void JointManager::add(Joint* _res) {
  ResourceManager<Joint>::add(_res);
  jnt_list_by_name_.insert(std::make_pair(_res->joint_name(), _res));

  if ((_res->owner_type() < 0) || (_res->joint_type() < 0)) {
    LOG_WARNING << "The joint '" << _res->getLabel()
        << "' must be configured inaccurately. LegType: "
        << _res->owner_type() << ", JntType: " << _res->joint_type();
    return;
  }

  if (jnt_list_by_type_.size() <= (size_t)_res->owner_type())
    jnt_list_by_type_.resize(_res->owner_type() + 1);
  if (jnt_list_by_type_[_res->owner_type()].size() <= (size_t)_res->joint_type())
    jnt_list_by_type_.resize(_res->joint_type() + 1);

  jnt_list_by_type_[_res->owner_type()][_res->joint_type()] = _res;
}

} /* namespace middleware */
