/*
 * label.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#include "label.h"

namespace middleware {

#define COMMA (".")

const MiiString             Label::null = "";
std::map<MiiString, Label*> Label::s_label_table_;

Label::Label(const std::string& l, const std::string& parent)
: label_(make_label(parent, l)) {
  label_table().insert(std::make_pair(label_, this));
}

Label::Label(const MiiString& _l, const Label& _obj)
: Label(_obj.getLabel(), _l) {
  ;
}

Label::Label(const MiiString& _l, Label* _obj)
: Label(_obj->getLabel(), _l) {
  ;
}

Label::~Label() {
  label_table().erase(label_);
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

} /* namespace middleware */
