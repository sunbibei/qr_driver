/*
 * label.h
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_LABEL_LABEL_H_
#define INCLUDE_SYSTEM_LABEL_LABEL_H_

#include <string>
#include <map>

#include "system/utils/log.h"

namespace middleware {

typedef std::string MiiString;

class Label {
public:
  const static MiiString null;

  Label(const MiiString& l, const MiiString& parent = "");
  Label(const MiiString&, const Label&);
  Label(const MiiString&, Label*);
  virtual ~Label();

  const MiiString& getLabel() { return label_; }
  const MiiString& getLabel() const { return label_; }

  static MiiString make_label  (const MiiString&, const MiiString&);
  static MiiString parent_label(const MiiString&);
  static void      split_label (MiiString, MiiString&, MiiString&);

  template<class _Hardware>
  static _Hardware* getHardwareByName(const MiiString&);

protected:
  MiiString label_;
  static std::map<MiiString, Label*>& label_table() { return s_label_table_; }

private:
  static std::map<MiiString, Label*> s_label_table_;
};





///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template method         ////////////
///////////////////////////////////////////////////////////////////////////////
template<class _Hardware>
_Hardware* Label::getHardwareByName(const MiiString& l) {
  auto& hw = label_table().find(l);
  if ((label_table().end() == hw)
      || (nullptr == dynamic_cast<_Hardware*>(hw->second))) {
    LOG_ERROR << "No Object: " << l;
    return nullptr;
  }

  LOG_INFO << "Got the hardware " << l << ", address " << hw->second;
  return static_cast<_Hardware*>(hw->second);
}

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_LABEL_LABEL_H_ */
