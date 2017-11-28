/*
 * cfg_reader.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#include "foundation/cfg_reader.h"
#include <boost/algorithm/string.hpp>

// Cancel the namespace middleware
// namespace middleware {

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
SINGLETON_IMPL_NO_CREATE(MiiCfgReader)

MiiCfgReader* MiiCfgReader::create_instance(const MiiString& file) {
  if (nullptr == instance_) {
    xml_doc = new TiXmlDocument();
    if (!xml_doc->LoadFile(file)) {
      LOG_FATAL << "Could not found the " << file
          << ", did you forget define the file?";
      return nullptr;
    }

    /*if (!xml_doc->Parse(file.c_str())) {
      LOG_FATAL << "Could not parse the configure file content.";
    }*/
    instance_ = new MiiCfgReader(file);
  } else {
    LOG_WARNING << "This method 'MiiCfgReader::create_instance' is called twice!";
  }

  return instance_;
}

MiiCfgReader::MiiCfgReader(const MiiString& file)
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
    MiiString __next_p = Label::make_label(__p, __next->Value());
    if (nullptr != __next->Attribute(attr.c_str()))
      cb(__next_p, __next->Attribute(attr.c_str()));
    else
      findAttr(__next, __next_p, attr, cb);
  }
}

TiXmlElement* findTag(TiXmlElement* __prefix, const MiiString& tag) {
  if (nullptr == __prefix) return nullptr;

  for (auto __tag = __prefix->FirstChildElement();
      nullptr != __tag; __tag = __tag->NextSiblingElement())
    return ((0 == tag.compare(__tag->Value())) ? __tag : findTag(__tag, tag));

  return nullptr;
}

TiXmlElement* findLabel(TiXmlElement* __root, MiiString __label) {
  MiiString __l;
  Label::split_label(__label, __label, __l);
  if (0 != __l.compare(__root->Value())) return nullptr;

  while (Label::null != __label) {
    Label::split_label(__label, __label, __l);
    __root = __root->FirstChildElement(__l);
    if (!__root) return nullptr;
  }

  return __root;
}

void MiiCfgReader::registerCallbackAndExcute(const MiiString& attr, Callback cb,
    const MiiString& __prefix) {
  auto root = root_;
  if (__prefix.empty()) root = root_;
  else root = findLabel(root, __prefix);
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
    MiiVector<char>& vals) {

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

bool MiiCfgReader::get_value(const MiiString& p, const MiiString& attr, MiiVector<unsigned char>& vals) {
  std::vector<char> vals_char;
  if (!get_value(p, attr, vals_char) || vals_char.empty()) return false;

  for (auto c : vals_char)
    vals.push_back(c);

  return true;
}

bool MiiCfgReader::get_value(const MiiString& p, const MiiString& attr, MiiVector<JntType>& vals) {
  std::vector<std::string> vals_str;
  if (!get_value(p, attr, vals_str)) return false;

  for (auto str : vals_str) {
    boost::to_lower(str);
    JntType type = JntType::UNKNOWN_JNT;
    if (0 == str.compare("yaw")) {
      type = JntType::YAW;
    } else if (0 == str.compare("hip")) {
      type = JntType::HIP;
    } else if (0 == str.compare("knee")) {
      type = JntType::KNEE;
    } else {
      LOG_WARNING << "Error the 'jnt' TAG(" << str << ") in the 'joint' TAG, "
          << "require 'yaw', 'knee' or 'hip'";
    }

    vals.push_back(type);
  }

  return true;
}

bool MiiCfgReader::get_value(const MiiString& p, const MiiString& attr, MiiVector<LegType>& vals) {
  std::vector<std::string> vals_str;
  if (!get_value(p, attr, vals_str)) return false;

  for (auto str : vals_str) {
    boost::to_lower(str);
    LegType type = LegType::UNKNOWN_LEG;
    if (0 == str.compare("fl")) {
      type = LegType::FL;
    } else if (0 == str.compare("fr")) {
      type = LegType::FR;
    } else if (0 == str.compare("hl")) {
      type = LegType::HL;
    } else if (0 == str.compare("hr")) {
      type = LegType::HR;
    } else {
      LOG_WARNING << "Error the 'leg' TAG(" << str << ") in the 'joint' TAG, "
          << "require 'hl', 'fr', 'hl' or 'hr'";
    }

    vals.push_back(type);
  }

  return true;
}

//} /* namespace middleware */
