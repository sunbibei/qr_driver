/*
 * joint.h
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_

#include "hw_unit.h"
#include "encoder.h"
#include "motor.h"

namespace middleware {

class Joint: public HwUnit {
public:
  typedef Motor::CmdType                CmdType;
  typedef Motor::CmdTypeSp              CmdTypeSp;
  typedef Encoder::StateType            StateType;
  typedef Encoder::StateTypeSp          StateTypeSp;

public:
  Joint(const std::string& name = "joint");
  virtual ~Joint();

  virtual bool init(TiXmlElement*) override;

  virtual HwStateSp getStataHandle() override;
  virtual HwCmdSp getCmdHandle() override;

  virtual HwStateSp getState() override;
  virtual HwCmdSp getCommand() override;
  virtual void setState(const HwState&) override;
  virtual void setCommand(const HwCommand&) override;

protected:
  // std::string actuators_;
  // std::string encoders_;
  boost::shared_ptr<Encoder> encoder_;
  boost::shared_ptr<Motor>   motor_;
};

typedef boost::shared_ptr<Joint> JointSp;

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_ */
