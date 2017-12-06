/*
 * cfg_reader.h
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_UTILS_CFG_READER_H_
#define INCLUDE_SYSTEM_UTILS_CFG_READER_H_

// #include <foundation/label.h>
// #include <foundation/utf.h>

#include "label.h"
#include "utf.h"

#include <vector>
#include <tinyxml.h>

// Cancel the namespace middleware
// namespace middleware {

class MiiCfgReader final {
  SINGLETON_DECLARE(MiiCfgReader, const MiiString&)

public:

  typedef void (*Callback)(const MiiString& __p, const MiiString& __attr_val);
  /**
   * @brief The Callback function for specific @__attr
   * @param __p        The parent of tag which contains the specific attribute.
   * @param __attr_val The value of __attr under the __p tag.
   */
  void regAttrCb(const MiiString& __attr, Callback, const MiiString& __prefix = "");
  /**
   * @brief The Callback function for specific @__tag
   * @param __p        The parent of tag which contains the specific attribute.
   * @param __attr_val The value of __attr under the __p tag.
   */
  // typedef void (*Callback1)(const MiiString& __p, TiXmlElement*);
  // void regTagCb(const MiiString& _tag, Callback1, const MiiString& _prefix = "");

public:
  /*!
   * @brief Add a new path into ENV $path, not a file name but a path.
   */
  static void add_path(const MiiString&);
  /*!
   * @brief Load and read a new configure file in the runtime.
   */
  bool add_config(const MiiString&);

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
  bool get_value(const MiiString& p, const MiiString& attr, _Type& val, const _Type& def);
  template<class _Type>
  bool get_value(const MiiString& p, const MiiString& attr, _Type& val);
  template<class _Type>
  bool get_value(const MiiString& p, const MiiString& attr, MiiVector<_Type>& vals);
  ////////////////// Template Specialization
  bool get_value(const MiiString& p, const MiiString& attr, MiiString&);
  bool get_value(const MiiString& p, const MiiString& attr, MiiVector<bool>&);
  bool get_value(const MiiString& p, const MiiString& attr, MiiVector<char>&);
  bool get_value(const MiiString& p, const MiiString& attr, MiiVector<unsigned char>&);
  bool get_value(const MiiString& p, const MiiString& attr, MiiVector<JntType>&);
  bool get_value(const MiiString& p, const MiiString& attr, MiiVector<LegType>&);

  /**
   *  @brief  Find the value of @p.@attr in the configure file.
   *          If not exist, This methods will be throwed an fatal exception.
   *  @param p[in]     The parent of attr
   *  @param attr[in]  Attribute name to locate.
   *  @param val[out]  The value associate to @attr.
   *  @return  Return true if this attribute has found, or return false.
  */
  template<class _Type>
  void get_value_fatal(const MiiString& p, const MiiString& attr, _Type& val);
  template<class _Type>
  void get_value_fatal(const MiiString& p, const MiiString& attr, MiiVector<_Type>& vals);

private:
  TiXmlDocument**      cfg_docs_;
  // TiXmlElement*        cfg_root_;

  size_t               n_config_;
  size_t               N_config_;

  MiiVector<MiiString> opened_cfg_file_;
  static MiiVector<MiiString>         s_cfg_paths_;
};






///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
// The helper method's forward declaration
bool __get_value_helper(TiXmlElement* __root, const MiiString& p,
    const MiiString& __attr, bool fatal, MiiString& __pAttr);

template<class _Type>
bool MiiCfgReader::get_value(const MiiString& p, const MiiString& attr, _Type& val, const _Type& def) {
  if (!get_value(p, attr, val))
    val = def;

  return true;
}

template<class _Type>
bool MiiCfgReader::get_value(const MiiString& p, const MiiString& attr, _Type& val) {
  std::vector<_Type> __vals;
  if ((!get_value(p, attr, __vals)) || (__vals.empty())) return false;

  val = __vals.back();
  return true;
}

template<class _Type>
bool MiiCfgReader::get_value(const MiiString& p, const MiiString& attr, std::vector<_Type>& vals) {
  MiiString __pAttr = "";
  for (size_t i = 0; i < n_config_; ++i) {
    auto _cfg_root = cfg_docs_[i]->RootElement();
    if (__get_value_helper(_cfg_root, p, attr, false, __pAttr)) break;
  }
  if (__pAttr.empty()) return false;

  std::stringstream __ss;
  __ss << __pAttr;
  vals.clear();
  _Type tmp;
  while (__ss >> tmp) vals.push_back(tmp);
  return !vals.empty();
}

template<class _Type>
void MiiCfgReader::get_value_fatal(const MiiString& p, const MiiString& attr, _Type& val) {
  std::vector<_Type> __vals;
  if ((!get_value(p, attr, __vals)) || (__vals.empty()))
    LOG_FATAL << "CfgReader can't found the configure '" << Label::make_label(p, attr) << "' in the configure file.";

  val = __vals.back();
}

template<class _Type>
void MiiCfgReader::get_value_fatal(const MiiString& p, const MiiString& attr, std::vector<_Type>& vals) {
  if (!get_value(p, attr, vals))
    LOG_FATAL << "CfgReader can't found the confgiure '" << Label::make_label(p, attr) << "' in the configure file.";
}

//} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_UTILS_CFG_READER_H_ */
