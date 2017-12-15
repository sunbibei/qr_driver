/*
 * motor.h
 *
 *  Created on: Dec 15, 2017
 *      Author: robot
 */

#ifndef INCLUDE_REPOSITORY_RESOURCE_MOTOR_H_
#define INCLUDE_REPOSITORY_RESOURCE_MOTOR_H_

#include <foundation/label.h>
#include <atomic>

namespace middleware {

class Motor: public Label {
  friend class LegNode;
public:
  Motor(const MiiString& _l = "motor");
  virtual bool init() override;

  virtual ~Motor();

public:
  void updateMotorCommand(double);

public:
  const MiiString&  motor_name() const;
  const MiiString&  joint_name() const;
  const JntType&    joint_type() const;
  const LegType&    leg_type()   const;
  const JntCmdType& cmd_mode()   const;

public:
  ///! About the state of motor
  const short  motor_position()               const;
  const short& motor_position_const_ref()     const;
  const short* motor_position_const_pointer() const;

  const short  motor_velocity()               const;
  const short& motor_velocity_const_ref()     const;
  const short* motor_velocity_const_pointer() const;

  const short  motor_torque()                 const;
  const short& motor_torque_const_ref()       const;
  const short* motor_torque_const_pointer()   const;

  ///! About the command of motor
  const short  motor_command()               const;
  const short& motor_command_const_ref()     const;
  const short* motor_command_const_pointer() const;

protected:
  /*!
   * Interface for communication layer, friend class LegNode.
   */
  void updateMotorPosition(short);
  void updateMotorVelocity(short);
  void updateMotorTorque  (short);

protected:
  MiiString           motor_name_;
  class Joint*        joint_handle_;
  class MotorState*   motor_state_;
  class MotorCommand* motor_cmd_;

  std::atomic_bool    new_command_;
};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_RESOURCE_MOTOR_H_ */
