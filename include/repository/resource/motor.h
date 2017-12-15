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
public:
  Motor(const MiiString&, const JntCmdType&);
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
  const double  motor_position()               const;
  const double& motor_position_const_ref()     const;
  const double* motor_position_const_pointer() const;

  const double  motor_velocity()               const;
  const double& motor_velocity_const_ref()     const;
  const double* motor_velocity_const_pointer() const;

  const double  motor_torque()                 const;
  const double& motor_torque_const_ref()       const;
  const double* motor_torque_const_pointer()   const;

  ///! About the command of motor
  const double  motor_command()               const;
  const double& motor_command_const_ref()     const;
  const double* motor_command_const_pointer() const;


protected:
  const class Joint*  joint_handle_;
  class MotorState*   motor_state_;
  class MotorCommand* motor_cmd_;

  std::atomic_bool    new_command_;
};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_RESOURCE_MOTOR_H_ */
