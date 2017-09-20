/*
 * joint.h
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_RESOURCES_JOINT_H_
#define INCLUDE_SYSTEM_RESOURCES_JOINT_H_

#include <system/foundation/label.h>
#include <atomic>

namespace middleware {

class Joint : public Label {
  friend class LegNode;
public:
  Joint(const MiiString& l = Label::null);
  // 妥协方案
  virtual bool init() override;
  ~Joint();

  const MiiString&   joint_name() const;
  const JntType&     joint_type() const;
  const LegType&     owner_type() const;
  /**
   * Interface for user layer.
   */
  // About joint state
  double        joint_position();
  const double& joint_position_const_ref();
  const double* joint_position_const_pointer();

  double        joint_velocity();
  const double& joint_velocity_const_ref();
  const double* joint_velocity_const_pointer();

  double        joint_torque();
  const double& joint_torque_const_ref();
  const double* joint_torque_const_pointer();

  ///! About joint command, This is only way that the user update the joint command.
  void updateJointCommand(double);
  void updateJointCommand(JntCmdType);
  void updateJointCommand(double, JntCmdType);

  double            joint_command();
  const double&     joint_command_const_ref();
  const double*     joint_command_const_pointer();

  JntCmdType        joint_command_mode();
  const JntCmdType& joint_command_mode_const_ref();
  const JntCmdType* joint_command_mode_const_pointer();

protected:
  /**
   * Interface for communication layer, friend class LegNode.
   */
  void updateJointPosition(double pos);
  // if has new command, return true and fill the Packet pointer
  std::atomic_bool new_command_; // the flag indicate whether has new command

protected:
  JntType             jnt_type_;
  LegType             leg_type_;
  MiiString           jnt_name_;
  // The private data structure
  class JointState*   joint_state_;
  class JointCommand* joint_command_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_RESOURCES_JOINT_H_ */
