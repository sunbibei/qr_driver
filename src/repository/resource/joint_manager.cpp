/*
 * joint_manager.cpp
 *
 *  Created on: Sep 1, 2017
 *      Author: silence
 */

#include <repository/resource/joint_manager.h>

namespace middleware {

SINGLETON_IMPL(JointManager)

JointManager::JointManager()
  : ResourceManager<Joint>() {
  jnt_list_by_type_.resize(LegType::N_LEGS);
  for (auto& leg : jnt_list_by_type_) {
    leg.resize(JntType::N_JNTS);
    for (auto& jnt : leg)
      jnt = nullptr;
  }
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
  jnt_list_by_type_[_res->owner_type()][_res->joint_type()] = _res;
  LOG_DEBUG << "The joint " << _res->getLabel() << " is received by JointManager";
}

JointManager::iterator JointManager::find(const MiiString& _n) {
  for (JointManager::iterator itr = JointManager::res_list_.begin();
      itr != JointManager::res_list_.end(); ++itr) {

    if (0 == _n.compare((*itr)->joint_name())) return itr;
  }

  return end();
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
  _c_p = jnt_list_by_type_[_owner][_type]->joint_position_const_pointer();
}
void JointManager::joint_velocity_const_pointer(LegType _owner, JntType _type, const double* & _c_p) {
  _c_p = jnt_list_by_type_[_owner][_type]->joint_velocity_const_pointer();
}
void JointManager::joint_torque_const_pointer  (LegType _owner, JntType _type, const double* & _c_p) {
  _c_p = jnt_list_by_type_[_owner][_type]->joint_torque_const_pointer();
}
// override
void JointManager::joint_position_const_pointer(const MiiString& _n, const double* & _c_p) {
  _c_p = jnt_list_by_name_[_n]->joint_position_const_pointer();
}
void JointManager::joint_velocity_const_pointer(const MiiString& _n, const double* & _c_p) {
  _c_p = jnt_list_by_name_[_n]->joint_velocity_const_pointer();
}
void JointManager::joint_torque_const_pointer(const MiiString& _n, const double* & _c_p) {
  _c_p = jnt_list_by_name_[_n]->joint_torque_const_pointer();
}
// override
void JointManager::joint_position_const_pointer(MiiVector<const double*>& _c_ps) {
  _c_ps.clear();
  for (auto jnt : res_list_) {
    _c_ps.push_back(jnt->joint_position_const_pointer());
  }
}
void JointManager::joint_velocity_const_pointer(MiiVector<const double*>& _c_ps) {
  _c_ps.clear();
  for (auto jnt : res_list_) {
    _c_ps.push_back(jnt->joint_velocity_const_pointer());
  }
}
void JointManager::joint_torque_const_pointer(MiiVector<const double*>& _c_ps) {
  _c_ps.clear();
  for (auto jnt : res_list_) {
    _c_ps.push_back(jnt->joint_torque_const_pointer());
  }
}

/*JointManager::iterator JointManager::find(const MiiString& _n) {
  for (auto itr = res_list_.begin(); itr != res_list_.end(); ++itr) {
    if (itr->joint_name() == _n) return itr;
  }
  return end();
}*/

} /* namespace middleware */