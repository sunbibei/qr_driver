/*
 * label.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#include <system/foundation/label.h>

namespace middleware {

#define COMMA (".")

const MiiString                      Label::null = "";
std::map<MiiString, Label::LabelPtr> Label::s_label_table_;

Label::Label(const MiiString& __l, const MiiString& __p)
: label_(make_label(__p, __l)) {
  /*std::cout << "table's address: " << &s_label_table_ << std::endl;
  std::cout << "Insert? address: " << this << std::endl;
  label_table().insert(std::make_pair(label_, this));
  std::cout << "After insert, size: " << s_label_table_.size() << std::endl;*/
}

Label::Label(const MiiString& _l, const Label& _obj)
: Label(_l, _obj.getLabel()) {
  ;
}

Label::Label(const MiiString& _l, LabelPtr _obj)
: Label(_l, _obj->getLabel()) {
  ;
}

Label::Label(const MiiString& _l, Label* _obj)
: Label(_l, _obj->getLabel()) {
  ;
}

Label::~Label() {
  // label_table().erase(label_);
}

MiiString Label::make_label(const MiiString& p, const MiiString& l) {
  return MiiString(p + COMMA + l);
}

MiiString Label::parent_label(const MiiString& l) {
  size_t p = l.rfind(COMMA);
  if (MiiString::npos == p)
    return null;
  else
    return l.substr(0, p);
}

void Label::split_label(MiiString l, MiiString& p, MiiString& v) {
  size_t pos = l.rfind(COMMA);
  if (MiiString::npos == pos) {
    p = null;
    v = l;
  } else {
    p = l.substr(0, pos);
    v = l.substr(pos+1, l.size());
  }
}

bool Label::init() { return true; }

} /* namespace middleware */
