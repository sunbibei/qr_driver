/*
 * joint_manager.cpp
 *
 *  Created on: Sep 1, 2017
 *      Author: silence
 */

#include <system/resources/joint_manager.h>

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

void JointManager::addJointCommand(LegType owner, JntType type, double val) {
  if (nullptr != jnt_list_by_type_[owner][type])
    jnt_list_by_type_[owner][type]->updateJointCommand(val);
}

void JointManager::addJointCommand(const MiiString& name, double val) {
  if (jnt_list_by_name_.end() != jnt_list_by_name_.find(name))
    jnt_list_by_name_[name]->updateJointCommand(val);
}

Joint* JointManager::getJointByType(LegType owner, JntType type) {
  return jnt_list_by_type_[owner][type];
}

void JointManager::joint_position_const_pointer(LegType _owner, JntType _type, const double* & _c_p) {
  ;
}
void JointManager::joint_velocity_const_pointer(LegType _owner, JntType _type, const double* & _c_p) {
  ;
}
void JointManager::joint_torque_const_pointer  (LegType _owner, JntType _type, const double* & _c_p) {
  ;
}
// override
void JointManager::joint_position_const_pointer(const MiiString& _n, const double* & _c_p) {
  ;
}
void JointManager::joint_velocity_const_pointer(const MiiString& _n, const double* & _c_p) {
  ;
}
void JointManager::joint_torque_const_pointer(const MiiString& _n, const double* & _c_p) {
  ;
}
// override
void JointManager::joint_position_const_pointer(MiiVector<const double*>&) {
  ;
}
void JointManager::joint_velocity_const_pointer(MiiVector<const double*>&) {
  ;
}
void JointManager::joint_torque_const_pointer(MiiVector<const double*>&) {
  ;
}

/*JointManager::iterator JointManager::find(const MiiString& _n) {
  for (auto itr = res_list_.begin(); itr != res_list_.end(); ++itr) {
    if (itr->joint_name() == _n) return itr;
  }
  return end();
}*/

} /* namespace middleware */
