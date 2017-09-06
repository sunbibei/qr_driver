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
public:
  static bool            create_instance(const MiiString&);
  static void            destroy_instance();
  static AutoInstanceor* instance();

public:
  /*template<class _Base>
  bool make_instance(MiiStringConstRef, _Base);*/

  /**
   * @brief Create an object about specific __type
   * @param __type The type of object to create
   * @param __l    The label for object
   */
  bool make_instance(const MiiString& __type, const MiiString& __l);

protected:
  AutoInstanceor(const MiiString&);
  virtual ~AutoInstanceor();

  static AutoInstanceor* instance_;

protected:
  static MiiMap<MiiString, Label::LabelPtr> s_inst_table_;
  class_loader::ClassLoader*                class_loader_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_UTILS_AUTO_INSTANCEOR_H_ */
