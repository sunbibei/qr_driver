/*
 * touchdown.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: bibei
 */

#include <middleware/hardware/touchdown.h>
#include <boost/algorithm/string.hpp>

namespace middleware {

TouchDown::TouchDown(const std::string& n)
  : HwUnit(n),
    leg_(LegType::FL) {
}

TouchDown::~TouchDown() {
  // Nothing to do here
}

bool TouchDown::init(TiXmlElement* root) {
  LOG_INFO << "[Joint: '" << root->Attribute("name") << "'] initialize start...";
  std::string tmp_str = root->Attribute("leg");
  boost::to_lower(tmp_str);
  if (0 == tmp_str.compare("fl")) {
    leg_ = LegType::FL;
  } else if (0 == tmp_str.compare("fr")) {
    leg_ = LegType::FR;
  } else if (0 == tmp_str.compare("hl")) {
    leg_ = LegType::HL;
  } else if (0 == tmp_str.compare("hr")) {
    leg_ = LegType::HR;
  } else {
    LOG_ERROR << "Error the 'leg' TAG(" << tmp_str << ") in the 'joint' TAG, "
        << "require 'fl', 'fr', 'hl' or 'hr'";
    return false;
  }
  return true;
}

HwStateSp TouchDown::getStateHandle() {
  return td_sp_;
}

bool TouchDown::requireCmdReg() {
  return false;
}

HwCmdSp   TouchDown::getCmdHandle() {
  LOG_WARNING << "There is not command handle in the 'TouchDown'";
  return nullptr;
}

HwStateSp TouchDown::getState() {
  return StateTypeSp(new TouchDown::StateType(td_sp_->data));
}

HwCmdSp   TouchDown::getCommand() {
  LOG_WARNING << "There is not command handle in the 'TouchDown'";
  return nullptr;
}

void TouchDown::setState(const HwState&) {
  LOG_WARNING << "The state handle is not assigned in the 'TouchDown'";
}

void TouchDown::setCommand(const HwCommand&) {
  LOG_WARNING << "The command handle is not assigned in the 'TouchDown'";
}

void TouchDown::publish() {
  ;
}

// for debug
void TouchDown::check() {
  ;
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::TouchDown, middleware::HwUnit)
