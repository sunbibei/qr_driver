/*
 * motor_pcan.h
 *
 *  Created on: Nov 1, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_MOTOR_PCAN_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_MOTOR_PCAN_H_

#include <system/platform/propagate/arm_pcan.h>

namespace middleware {

class MotorPcan: public ArmPcan {
public:
  MotorPcan(const MiiString& l = "motor_pcan");
  virtual bool init() override;

  virtual ~MotorPcan();

public:
  virtual bool write(const class Packet&) override;
  virtual bool read (class Packet&)       override;

protected:
  short X_[MAX_NODE_NUM][JntType::N_JNTS];
  short U_[MAX_NODE_NUM][JntType::N_JNTS];
  short T_[MAX_NODE_NUM][JntType::N_JNTS];
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_PROPAGATE_MOTOR_PCAN_H_ */
