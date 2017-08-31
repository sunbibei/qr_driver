/*
 * touchdown.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: bibei
 */

#include "system/utils/cfg_reader.h"

#include <middleware/hardware/touchdown.h>
#include <boost/algorithm/string.hpp>
#include <tinyxml.h>

namespace middleware {

struct TDState {
  TDState(double d = 0) : data(0) {}
  double data;
};

TouchDown::TouchDown(MiiStringConstRef l)
  : Label(l),leg_type_(LegType::UNKNOWN_LEG),
    msg_id_(0), td_state_(new TDState),
    scale_(0), offset_(0) {
  // The implement of init() should be here.
  ;
}

TouchDown::~TouchDown() {
  // Nothing to do here
  if (nullptr != td_state_) {
    delete td_state_;
    td_state_ = nullptr;
  }
}

bool TouchDown::init() {
  auto cfg = MiiCfgReader::instance();

  cfg->get_value_fatal(getLabel(), "msg_id", msg_id_);
  cfg->get_value_fatal(getLabel(), "msg_id", scale_);
  cfg->get_value_fatal(getLabel(), "msg_id", offset_);

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

inline void TouchDown::updateTouchdownState(short _count) {
  td_state_->data = _count * scale_ + offset_;
}

inline double TouchDown::touchdown_data() {
  return td_state_->data;
}

inline const LegType& TouchDown::leg_type() const
{ return leg_type_; }

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::TouchDown, middleware::Label)
