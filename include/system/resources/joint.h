/*
 * joint.h
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_RESOURCES_JOINT_H_
#define INCLUDE_SYSTEM_RESOURCES_JOINT_H_

#include <system/foundation/label.h>

namespace middleware {

class Joint : public Label {
  friend class LegNode;
public:
  Joint(const MiiString& l = Label::null);
  // 妥协方案
  virtual bool init() override;

  const MiiString&   joint_name() const;
  const JntType&     joint_type() const;
  const LegType&     owner_type() const;
  /**
   * Interface for user layer.
   */
  // About joint state
  double joint_position();
  double joint_velocity();
  double joint_torque();
  const double* joint_position_const_pointer();
  const double* joint_velocity_const_pointer();
  const double* joint_torque_const_pointer();

  // About joint command
  void updateJointCommand(double);
  void updateJointCommand(double, JntCmdType);
  double joint_command();

  void changeJointCommandMode(JntCmdType);
  JntCmdType joint_command_mode();

protected:
  /**
   * Interface for communication layer, friend class LegNode.
   */
  // The velocity will be change after updating joint position
  // position = count * scale + offset
  void updateJointPosition(short count);
  // if has new command, return true and fill the Packet pointer
  bool new_command_; // the flag indicate whether has new command
  // bool new_command(class Packet*);
  // position = count * scale + offset

protected:
  JntType             jnt_type_;
  LegType             leg_type_;
  std::string         jnt_name_;
  double              scale_;
  double              offset_;

  unsigned char       msg_id_;
  class JointState*   joint_state_;
  class JointCommand* joint_command_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_RESOURCES_JOINT_H_ */
