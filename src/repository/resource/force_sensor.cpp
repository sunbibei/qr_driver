/*
 * touchdown.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: bibei
 */

#include "system/foundation/cfg_reader.h"

#include <boost/algorithm/string.hpp>
#include <repository/resource/force_sensor.h>
#include <tinyxml.h>

namespace middleware {

struct ForceState {
  ForceState(double d = 0) : data(0) {}
  double data;
};

ForceSensor::ForceSensor(const MiiString& l)
  : Label(l),leg_type_(LegType::UNKNOWN_LEG),
    td_state_(new ForceState),
    scale_(0), offset_(0) {
  // The implement of init() should be here.
  ;
}

ForceSensor::~ForceSensor() {
  // Nothing to do here
  if (nullptr != td_state_) {
    delete td_state_;
    td_state_ = nullptr;
  }
}

bool ForceSensor::init() {
  auto cfg = MiiCfgReader::instance();

  cfg->get_value(getLabel(), "msg_id", scale_);
  cfg->get_value(getLabel(), "msg_id", offset_);

  std::string tmp_str;
  cfg->get_value_fatal(getLabel(), "leg", tmp_str);
  boost::to_lower(tmp_str);
  if (0 == tmp_str.compare("fl")) {
    leg_type_ = LegType::FL;
  } else if (0 == tmp_str.compare("fr")) {
    leg_type_ = LegType::FR;
  } else if (0 == tmp_str.compare("hl")) {
    leg_type_ = LegType::HL;
  } else if (0 == tmp_str.compare("hr")) {
    leg_type_ = LegType::HR;
  } else {
    LOG_WARNING << "Error the 'leg' TAG(" << tmp_str << ") in the 'joint' TAG, "
        << "require 'hl', 'fr', 'hl' or 'hr'";
    return false;
  }

  return true;
}

void ForceSensor::updateForceCount(short _count) {
  td_state_->data = _count * scale_ + offset_;
}

double ForceSensor::force_data() {
  return td_state_->data;
}

const double* ForceSensor::force_data_const_pointer() {
  return &(td_state_->data);
}

const double& ForceSensor::force_data_const_ref() {
  return td_state_->data;
}

const LegType& ForceSensor::leg_type() const
{ return leg_type_; }

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::ForceSensor, middleware::Label)
