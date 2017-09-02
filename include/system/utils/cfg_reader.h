/*
 * cfg_reader.h
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_UTILS_CFG_READER_H_
#define INCLUDE_SYSTEM_UTILS_CFG_READER_H_

#include <system/utils/utf.h>
#include "system/label/label.h"

#include <vector>
#include <tinyxml.h>

namespace middleware {

class MiiCfgReader final {
public:
  static MiiCfgReader* instance();
  /**
   * Given the configure file name and create the instance.
   */
  static bool create_instance(ConstRef<MiiString> file);
  static bool destroy_instance();

public:
  /**
   * @brief The Callback function for specific @__attr
   * @param __p        The parent of tag which contains the specific attribute.
   * @param __attr_val The value of __attr under the __p tag.
   */
  typedef void (*Callback)(ConstRef<MiiString> __p, ConstRef<MiiString> __attr_val);
  void registerCallbackAndExcute(ConstRef<MiiString> __attr, Callback);

public:
  /**
   *  @brief  Find the value of @p.@attr in the configure file.
   *  @param p[in]     The parent of attr
   *  @param attr[in]  Attribute name to locate.
   *  @param val[out]  The value associate to @attr.
   *  @param def[in]   default value.
   *  @return  Return true if this attribute has found, or return false.
  */
  template<class _Type>
  bool get_value(ConstRef<MiiString> p, ConstRef<MiiString> attr, _Type& val, const _Type& def);
  template<class _Type>
  bool get_value(ConstRef<MiiString> p, ConstRef<MiiString> attr, _Type& val);
  template<class _Type>
  bool get_value(ConstRef<MiiString> p, ConstRef<MiiString> attr, MiiVector<_Type>& vals);
  ////////////////// Template Specialization
  bool get_value(ConstRef<MiiString> p, ConstRef<MiiString> attr, MiiVector<bool>&);
  bool get_value(ConstRef<MiiString> p, ConstRef<MiiString> attr, MiiVector<char>&);
  bool get_value(ConstRef<MiiString> p, ConstRef<MiiString> attr, MiiVector<unsigned char>&);

  /**
   *  @brief  Find the value of @p.@attr in the configure file.
   *          If not exist, This methods will be throwed an fatal exception.
   *  @param p[in]     The parent of attr
   *  @param attr[in]  Attribute name to locate.
   *  @param val[out]  The value associate to @attr.
   *  @return  Return true if this attribute has found, or return false.
  */
  template<class _Type>
  void get_value_fatal(ConstRef<MiiString> p, ConstRef<MiiString> attr, _Type& val);
  template<class _Type>
  void get_value_fatal(ConstRef<MiiString> p, ConstRef<MiiString> attr, MiiVector<_Type>& vals);

private:
  MiiCfgReader(ConstRef<MiiString> file);
  virtual ~MiiCfgReader();
  static MiiCfgReader* instance_;
  TiXmlElement* root_;
};






///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
// The helper method's forward declaration
bool __get_value_helper(TiXmlElement* __root, ConstRef<MiiString> p,
    ConstRef<MiiString> __attr, bool fatal, const char** __pAttr);

template<class _Type>
bool MiiCfgReader::get_value(ConstRef<MiiString> p, ConstRef<MiiString> attr, _Type& val, const _Type& def) {
  if (!get_value(p, attr, val))
    val = def;

  return true;
}
template<class _Type>
bool MiiCfgReader::get_value(ConstRef<MiiString> p, ConstRef<MiiString> attr, _Type& val) {
  std::vector<_Type> __vals;
  if ((!get_value(p, attr, __vals)) || (__vals.empty())) return false;

  val = __vals.back();
  return true;
}

template<class _Type>
bool MiiCfgReader::get_value(ConstRef<MiiString> p, ConstRef<MiiString> attr, std::vector<_Type>& vals) {
  const char* __pAttr = nullptr;
  if (!__get_value_helper(root_, p, attr, false, &__pAttr)) return false;

  std::stringstream __ss;
  __ss << __pAttr;
  vals.clear();
  _Type tmp;
  while (__ss >> tmp) vals.push_back(tmp);
  return !vals.empty();
}

template<class _Type>
void MiiCfgReader::get_value_fatal(ConstRef<MiiString> p, ConstRef<MiiString> attr, _Type& val) {
  std::vector<_Type> __vals;
  if ((!get_value(p, attr, __vals)) || (__vals.empty()))
    LOG_FATAL << "CfgReader can't found the confiure '" << Label::make_label(p, attr) << "' in the configure file.";

  val = __vals.back();
}

template<class _Type>
void MiiCfgReader::get_value_fatal(ConstRef<MiiString> p, ConstRef<MiiString> attr, std::vector<_Type>& vals) {
  if (!get_value(p, attr, vals))
    LOG_FATAL << "CfgReader can't found the confiure '" << Label::make_label(p, attr) << "' in the configure file.";
}

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_UTILS_CFG_READER_H_ */
