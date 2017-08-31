/*
 * touchdown.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: bibei
 */

#include <middleware/hardware/touchdown.h>
#include <boost/algorithm/string.hpp>
#include <tinyxml.h>

namespace middleware {

struct TDState {
  TDState(double d = 0) : data(0) {}
  double data;
};

TouchDown::TouchDown(TiXmlElement* root)
  : Label(root->Attribute("name"),
    td_state_(new TDState),
    scale_(0), offset_(0) {
  std::string tmp_str = root->Attribute("msg_id");
  std::string template_str;
  if (('0' == tmp_str[0]) && ('x' == tmp_str[1])) {
    // Hex to id
    template_str = "0x%x";
  } else {
    template_str = "%d";
  }
  unsigned int id;
  sscanf(tmp_str.c_str(), template_str.c_str(), &id);
  msg_id_ = id;

  root->Attribute("scale",  &scale_);
  root->Attribute("offset", &offset_);
}

TouchDown::~TouchDown() {
  // Nothing to do here
  if (nullptr != td_state_) {
    delete td_state_;
    td_state_ = nullptr;
  }
}

inline void TouchDown::updateTouchdownState(short _count) {
  td_state_->data = _count * scale_ + offset_;
}

inline double TouchDown::touchdown_data() {
  return td_state_->data;
}

} /* namespace middleware */
