/*
 * encoder.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include "middleware/hardware/encoder.h"
#include "middleware/util/log.h"

namespace middleware {

EncoderState::EncoderState(double pos, double vel)
  : pos_(pos), vel_(vel),
    previous_time_(std::chrono::high_resolution_clock::now())
{ };

EncoderState::~EncoderState()
{ };

Encoder::Encoder(const std::string& name)
  : HwUnit(name), state_(new StateType)
{ };

Encoder::~Encoder()
{ };

bool Encoder::init(TiXmlElement* para) {
  if (nullptr == para->Attribute("name")) {
    LOG_ERROR << "Can't found the 'name' TAG in the 'parameter' TAG";
    return false;
  }
  hw_name_ = para->Attribute("name");
  return true;
}


HwStateSp Encoder::getStataHandle() {
  return state_;
}

HwStateSp Encoder::getState() {
  return HwStateSp(new StateType(state_->pos_, state_->vel_));
}

void Encoder::check() {
  LOG_WARNING << "================check================";
  LOG_INFO << "NAME: " << hw_name_;
  LOG_INFO << "TYPE\tADDR\tCOUNT";
  LOG_INFO << "STATE\t" << state_.get() << "\t" << state_.use_count();
  LOG_WARNING << "=====================================";
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(middleware::Encoder, middleware::HwUnit)
