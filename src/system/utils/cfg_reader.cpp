/*
 * cfg_reader.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#include "cfg_reader.h"

#include <boost/algorithm/string.hpp>

namespace middleware {

bool __recurse_get_value(TiXmlElement* __ele, std::vector<MiiString> __path,
                      const MiiString& attr, const char** __val) {
  MiiString tag = __path.back();
  __path.pop_back();

  for (TiXmlElement* __next_ele = __ele->FirstChildElement(tag);
      nullptr != __next_ele; __next_ele = __next_ele->NextSiblingElement(tag)) {
    if (__path.empty()) {
      if (nullptr != __next_ele->Attribute(attr.c_str())) {
        *__val = __next_ele->Attribute(attr.c_str());
        return true;
      }
    } else {
      if (__recurse_get_value(__next_ele, __path, attr, __val)) {
        return true;
      }
    }
  }

  return false;
}

bool __get_value_helper(TiXmlElement* __root, const MiiString& p,
    const MiiString& __attr, bool fatal, const char** __pAttr) {
  std::vector<MiiString> __ps;
  MiiString __p = p;
  while (Label::null != __p) {
    MiiString __l;
    Label::split_label(__p, __p, __l);
    if (Label::null != __l) __ps.push_back(__l);
  }

  if ((__ps.empty()) || (0 == __ps.back().compare(__root->Value()))) {
    if (__ps.size() > 1) {
      // There are not only one element '__root->Value()' in the __ps
      __ps.pop_back();
    } else {
      if (nullptr == __root->Attribute(__attr.c_str()))
        return false;
      else
        *__pAttr = __root->Attribute(__attr.c_str());
      return true;
    }
  }

  // __ps.pop_back();
  if (!__recurse_get_value(__root, __ps, __attr, __pAttr)) {
    if (fatal) LOG_FATAL << "Can't found the parameter '" << Label::make_label(p, __attr) << "'";
    else return false;
  }

  return true;
}


TiXmlDocument* xml_doc = nullptr;
MiiCfgReader* MiiCfgReader::instance_ = nullptr;

MiiCfgReader* MiiCfgReader::instance() {
  if (nullptr == instance_) {
    std::cout << "This should be called after create_instance()" << std::endl;
    return nullptr;
  }

  return instance_;
}

bool MiiCfgReader::create_instance(const std::string& file) {
  if (nullptr != instance_) {
    std::cout << "Create the CfgReader twice!" << std::endl;
    return false;
  }
  xml_doc = new TiXmlDocument();
  if (!xml_doc->LoadFile(file)) {
    std::cout << "Could not found the " << file
        << ", did you forget define the file?" << std::endl;
    return false;
  }

  instance_ = new MiiCfgReader(file);
  return true;
}

bool MiiCfgReader::destroy_instance() {
  if (nullptr != instance_) {
    delete instance_;
    instance_ = nullptr;
  }

  return true;
}

MiiCfgReader::MiiCfgReader(const std::string& file)
: root_(xml_doc->RootElement()) {
  ;
}

MiiCfgReader::~MiiCfgReader() {
  if (nullptr != root_)
    root_ = nullptr;

  if (nullptr != xml_doc) {
    delete xml_doc;
    xml_doc = nullptr;
  }
}

void findAttr(TiXmlElement* __curr, const MiiString& __p,
    const MiiString& attr, MiiCfgReader::Callback cb) {

  for (auto __next = __curr->FirstChildElement();
      nullptr != __next; __next = __next->NextSiblingElement()) {
    if (nullptr != __next->Attribute(attr.c_str()))
      cb(__p, __next->Value());
    else
      findAttr(__next, Label::make_label(__p, __next->Value()), attr, cb);
  }
}

void MiiCfgReader::registerCallbackAndExcute(const MiiString& attr, Callback cb) {
  if (nullptr != root_->Attribute(attr.c_str()))
    cb(Label::null, root_->Value());

  findAttr(root_, root_->Value(), attr, cb);
}

bool MiiCfgReader::get_value(const MiiString& p, const MiiString& attr,
    std::vector<bool>& vals) {

  std::vector<std::string> vec_str;
  if (!get_value(p, attr, vec_str)) return false;

  vals.clear();
  for (auto& tmp : vec_str) {
    boost::to_lower(tmp);
    if (0 == tmp.compare("false"))
      vals.push_back(false);
    else if (0 == tmp.compare("true"))
      vals.push_back(true);
    else
      LOG_WARNING << "What a fucking configure? '" << tmp << "'";
  }
  return !vals.empty();
}

bool MiiCfgReader::get_value(const MiiString& p, const MiiString& attr,
    std::vector<char>& vals) {

  std::vector<std::string> vals_str;
  if (!get_value(p, attr, vals_str)) return false;

  std::string template_str;
  for (const auto& str : vals_str) {
    if (('0' == str[0]) && ('x' == str[1])) {
      // Hex to id
      template_str = "0x%x";
    } else {
      template_str = "%d";
    }
    unsigned int val_i;
    sscanf(str.c_str(), template_str.c_str(), &val_i);
    vals.push_back((char)val_i);
  }

  return true;
}

} /* namespace middleware */
