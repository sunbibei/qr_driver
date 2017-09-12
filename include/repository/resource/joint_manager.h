/*
 * joint_manager.h
 *
 *  Created on: Sep 1, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_ROBOT_JOINT_MANAGER_H_
#define INCLUDE_SYSTEM_ROBOT_JOINT_MANAGER_H_

#include <repository/resource/joint.h>
#include <system/platform/internal/resource_manager.h>
#include <vector>
#include <string>
#include <map>

namespace middleware {

class JointManager: public internal::ResourceManager<Joint> {
  SINGLETON_DECLARE(JointManager)

  friend class Joint;
public:
///////////////////////////////////////////////////////////////////////////////
//////////////////////        The Common Methods         //////////////////////
///////////////////////////////////////////////////////////////////////////////
  /**
   * @brief This method finds a specific named joint, return iteration if successful,
   *        return end() if it has not found.
   */
  iterator find(const MiiString& _n);
///////////////////////////////////////////////////////////////////////////////
////////////////////        The Common Interfaces         /////////////////////
///////////////////////////////////////////////////////////////////////////////
  /**
   * @brief These methods offer the interfaces what add the joint commands, get
   *        the joint states, or get the pointer of Joint object.
   */
  void   addJointCommand(LegType, JntType, double);
  // void   addJointCommand(const MiiVector<LegType>&, const MiiVector<JntType>&, const MiiVector<double>&);
  void   addJointCommand(const MiiString&, double);
  // void   addJointCommand(const MiiString&, const MiiVector<double>&);
  Joint* getJointByType(LegType, JntType);

///////////////////////////////////////////////////////////////////////////////
////////////////////        The Advanced Interfaces        ////////////////////
///////////////////////////////////////////////////////////////////////////////
  /**
   * @brief These methods offer some interfaces, an example as follows.
   *        auto jnt_m_ = JointManager::instance();
   *        const double* fl_yaw_pos = nullptr;
   *        jnt_m_->joint_position_const_pointer(LegType::FL, JntType::YAW, fl_yaw_pos);
   *        // OR call this method
   *        jnt_m_->joint_position_const_pointer('fl_yaw', fl_yaw_pos);
   *        // Now, the fl_yaw_pos always keep the newest value of the position of joint 'fl_yaw'
   *        std::vector<const double*> jnt_pos_;
   *        jnt_pos_.resize(jnt_names.size());
   *        // Let's pretend that the argument jnt_names contains the all of joint name you need.
   *        for (int i = 0; i < jnt_names.size(); ++i)
   *          joint_position_const_pointer(jnt_names[i], jnt_pos_[i]);
   */
  void joint_position_const_pointer(LegType _owner, JntType _type, const double* & _c_p);
  void joint_velocity_const_pointer(LegType _owner, JntType _type, const double* & _c_p);
  void joint_torque_const_pointer  (LegType _owner, JntType _type, const double* & _c_p);
  // override
  void joint_position_const_pointer(const MiiString& _n, const double* & _c_p);
  void joint_velocity_const_pointer(const MiiString& _n, const double* & _c_p);
  void joint_torque_const_pointer  (const MiiString& _n, const double* & _c_p);
  // override
  void joint_position_const_pointer(MiiVector<const double*>&);
  void joint_velocity_const_pointer(MiiVector<const double*>&);
  void joint_torque_const_pointer  (MiiVector<const double*>&);

protected:
  // Owner Size * Joint Size
  std::vector<std::vector<Joint*>>   jnt_list_by_type_;
  std::map<std::string, Joint*>      jnt_list_by_name_;

protected:
  virtual void add(Joint* _res) override;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_JOINT_MANAGER_H_ */
