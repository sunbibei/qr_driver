/*
 * joint.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#include "middleware/hardware/joint.h"
#include "middleware/util/log.h"

namespace middleware {

Joint::Joint(const std::string& name): HwUnit(name) { }

Joint::~Joint() { }

HwStateSp Joint::getStataHandle() {
  return encoder_->getStataHandle();
}

HwCmdSp Joint::getCmdHandle() {
  return motor_->getCmdHandle();
}

HwStateSp Joint::getState() {
  return encoder_->getState();
}

HwCmdSp Joint::getCommand() {
  return motor_->getCommand();
}

void Joint::setCommand(const HwCommand& c) {
  motor_->setCommand(c);
}

void Joint::setState(const HwState& s) {
  encoder_->setState(s);
}

bool Joint::init(TiXmlElement* jnt_root) {
  if (nullptr == jnt_root) {
    LOG_ERROR << "Can't found the 'parameter' TAG in the 'Joint' TAG";
    return false;
  }

  if (nullptr == jnt_root->Attribute("name"))
    LOG_WARNING << "Can't found the 'name' TAG in the 'parameter' TAG";
  else
    hw_name_ = jnt_root->FirstChildElement("parameter")->Attribute("name");

  TiXmlElement* act = jnt_root->FirstChildElement("actuator");
  if (nullptr == act) {
    LOG_FATAL << "Could not found the TAG('actuator') in the 'joint', "
        << "Did you forget define the file?";
    return false;
  }
  TiXmlElement* enc = jnt_root->FirstChildElement("encoder");
  if (nullptr == act) {
    LOG_FATAL << "Could not found the TAG('encoder') in the 'joint', "
        << "Did you forget define the file?";
    return false;
  }
  motor_.reset(new Motor());
  motor_->init(act);

  encoder_.reset(new Encoder());
  encoder_->init(enc);

  return true;
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(middleware::Joint, middleware::HwUnit)
