/*
 * encoder.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include "middleware/hardware/encoder.h"
#include "middleware/util/log.h"

#include "middleware/propagate/proto/dragon.pb.h"

namespace middleware {

EncoderState::EncoderState(LegType leg, JntType jnt, double pos, double vel)
  : leg_(leg), jnt_(jnt),
    pos_(pos), vel_(vel),
    previous_time_(std::chrono::high_resolution_clock::now())
{ };

EncoderState::~EncoderState()
{ };

bool EncoderState::update(const Feedback* fb) {

  if ((nullptr == fb) || (FbType::JOINT_STATES != fb->idx())
      || !fb->has_joint_states()
      || JntType::N_JNTS != fb->joint_states().pos_size()) {
    return false;
  }
  const auto& jnts = fb->joint_states();

  vel_ = (jnts.pos(jnt_) - pos_)
      / std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - previous_time_).count();
  pos_ = jnts.pos(jnt_);
  previous_time_ = std::chrono::high_resolution_clock::now();

  return true;
}

Encoder::Encoder(const std::string& name)
  : HwUnit(name), state_(nullptr)
{ };

Encoder::~Encoder()
{ };

bool Encoder::init(TiXmlElement* para) {
  if (nullptr == para->Attribute("name")) {
    LOG_ERROR << "Can't found the 'name' TAG in the 'parameter' TAG";
    return false;
  }
  hw_name_ = para->Attribute("name");

  std::string tmp_str = "";
  LegType leg;
  if (nullptr != para->Attribute("leg")) {
    tmp_str = para->Attribute("leg");
  } else {
    LOG_ERROR << "Can't found the 'leg' TAG in the 'parameter' TAG";
    return false;
  }
  if (0 == tmp_str.compare("fl") || 0 == tmp_str.compare("FL")) {
    leg = LegType::FL;
  } else if (0 == tmp_str.compare("fr") || 0 == tmp_str.compare("FR")) {
    leg = LegType::FR;
  } else if (0 == tmp_str.compare("hl") || 0 == tmp_str.compare("HL")) {
    leg = LegType::HL;
  } else if (0 == tmp_str.compare("hr") || 0 == tmp_str.compare("HR")) {
    leg = LegType::HR;
  } else {
    LOG_ERROR << "Error the 'leg' TAG in the 'parameter' TAG";
    return false;
  }
  tmp_str = "";
  JntType jnt;
  if (nullptr != para->Attribute("jnt")) {
    tmp_str = para->Attribute("jnt");
  } else {
    LOG_ERROR << "Can't found the 'jnt' TAG in the 'parameter' TAG";
    return false;
  }
  if (0 == tmp_str.compare("yaw") || 0 == tmp_str.compare("YAW")) {
    jnt = JntType::YAW;
  } else if (0 == tmp_str.compare("knee") || 0 == tmp_str.compare("KNEE")) {
    jnt = JntType::KNEE;
  } else if (0 == tmp_str.compare("hip") || 0 == tmp_str.compare("HIP")) {
    jnt = JntType::HIP;
  } else {
    LOG_ERROR << "Error the 'jnt' TAG in the 'parameter' TAG";
    return false;
  }

  state_.reset(new StateType(leg, jnt));

  return true;
}


HwStateSp Encoder::getStataHandle() {
  return state_;
}

HwStateSp Encoder::getState() {
  return HwStateSp(new StateType(state_->leg_, state_->jnt_, state_->pos_, state_->vel_));
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
