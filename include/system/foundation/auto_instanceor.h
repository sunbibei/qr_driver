/*
 * auto_instanceor.h
 *
 *  Created on: Aug 31, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_UTILS_AUTO_INSTANCEOR_H_
#define INCLUDE_SYSTEM_UTILS_AUTO_INSTANCEOR_H_

#include <map>
#include <boost/shared_ptr.hpp>
#include <class_loader/class_loader.h>
#include <system/foundation/label.h>

namespace middleware {

class AutoInstanceor {
  SINGLETON_DECLARE(AutoInstanceor, const MiiString&)

public:
  /*template<class _Base>
  bool make_instance(MiiStringConstRef, _Base);*/

  /**
   * @brief Create an object about specific __type
   * @param __p    The tag which contains the type
   * @param __type The type of object to create
   */
  bool make_instance(const MiiString& __p, const MiiString& __type);

protected:
  static MiiMap<MiiString, Label::LabelPtr> s_inst_table_;
  class_loader::ClassLoader*                class_loader_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_UTILS_AUTO_INSTANCEOR_H_ */
