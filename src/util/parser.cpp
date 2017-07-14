/*
 * robot_parser.cpp
 *
 *  Created on: Dec 3, 2016
 *      Author: silence
 */


#include "middleware/util/parser.h"

#include "middleware/util/log.h"
#include "middleware/middleware.h"
#include "middleware/util/composite.h"
#include "middleware/hardware/hw_unit.h"
#include "middleware/propagate/propagate.h"

namespace middleware {

TiXmlElement*               Parser::xml_root_      = nullptr;
class_loader::ClassLoader*  Parser::propa_loader_  = nullptr;
class_loader::ClassLoader*  Parser::unit_loader_   = nullptr;
TiXmlElement*               Parser::tmp_xml_ele_   = nullptr;

bool Parser::parse() {
  if (!Parser::init()) {
    LOG_ERROR << "RobotParser has initialized fail";
    return false;
  }

  return (parsePropagates() && parseHwUnits());
}

#ifndef ROS_BUILD
bool Parser::parse(const std::string& filename) {
  if (!Parser::init(filename)) {
    LOG_ERROR << "RobotParser has initialized fail";
    return false;
  }

  return (parsePropagates() && parseHwUnits());
}
/**
 * 完成解析函数MAP的初始化， 以及别的初始化工作
 */
bool Parser::init(const std::string& filename) {
  // 初始化XML文档相关内容
  TiXmlDocument* xml_doc = new TiXmlDocument();
  if (!xml_doc->LoadFile(filename)) {
    LOG_ERROR << "Could not found the "<< filename << ", did you forget define the file?";
    return false;
  }

  return initVariants(xml_doc);
}

bool Parser::initVariants(TiXmlDocument* xml_doc) {
  xml_root_ = xml_doc->RootElement();  // Robot
  // Initialize the class_loader
  TiXmlElement* lib_root = xml_root_->FirstChildElement("lib_paths");
  if (nullptr == lib_root) {
    LOG_ERROR << "No define the 'lib_paths' child element in configure";
    return false;
  }
  TiXmlElement* propa_lib = lib_root->FirstChildElement("propagate");
  TiXmlElement* unit_lib  = lib_root->FirstChildElement("hw_unit");
  if ((nullptr == propa_lib) || (nullptr == propa_lib->Attribute("path"))) {
    LOG_ERROR << "No define the lib paths of propagate";
    return false;
  } else {
    propa_loader_ = new class_loader::ClassLoader(propa_lib->Attribute("path"));
  }
  if ((nullptr == unit_lib) || (nullptr == unit_lib->Attribute("path"))) {
    LOG_ERROR << "No define the lib paths of unit";
    return false;
  } else {
    unit_loader_ = new class_loader::ClassLoader(unit_lib->Attribute("path"));
  }

  return true;
}
#endif

bool Parser::init() {
  std::string xml;
  // if (!nh.getParam("configure", configure)) {
  if (!ros::param::get("~configure", xml)) {
    LOG_FATAL << "No 'configure' parameter in ROS Parameter Server, "
        << "Did you forget define this parameter";
        //<< nh.getNamespace();
    return false;
  }

  std::string lib_propa;
  if (!ros::param::get("~lib_propagate", lib_propa)) {
    LOG_FATAL << "No 'lib_propagate' parameter in ROS Parameter Server, "
        << "Did you forget define this parameter";
        //<< nh.getNamespace();
    return false;
  }

  std::string lib_hwunit;
  if (!ros::param::get("~lib_hwunit", lib_hwunit)) {
    LOG_FATAL << "No 'lib_hwunit' parameter in ROS Parameter Server, "
        << "Did you forget define this parameter";
        //<< nh.getNamespace();
    return false;
  }

  // LOG_INFO << configure;
  // 初始化XML文档相关内容
  TiXmlDocument* xml_doc = new TiXmlDocument();
  xml_doc->Parse(xml.c_str());
  propa_loader_ = new class_loader::ClassLoader(lib_propa);
  unit_loader_  = new class_loader::ClassLoader(lib_hwunit);
  return true;
}

bool Parser::checkPropagatesFormat() {
  if (nullptr == propa_loader_) {
    LOG_FATAL << "propa_loader is nullptr!";
    return false;
  }
  tmp_xml_ele_ = xml_root_->FirstChildElement("propagates");
  if (nullptr == tmp_xml_ele_) {
    LOG_FATAL << "No 'propagates' parameter in configure content, "
            << "Did you forget define this parameter";
    return false;
  }

  // auto& propa = Middleware::getInstance()->propagate_;
  if (nullptr == tmp_xml_ele_->Attribute("name")) {
    LOG_WARNING << "Could not found the 'name' attribute of the 'propagates' block, "
        << "using the default name 'propagates'";
    tmp_xml_ele_->SetAttribute("name", "propagates");
  }
  return true;
}

bool Parser::parsePropagates() {

  if (!checkPropagatesFormat()) return false;

  tmp_xml_ele_ = xml_root_->FirstChildElement("propagates");
  LOG_INFO << "Assemble propagates: '" << tmp_xml_ele_->Attribute("name") << "'";
  Middleware::instance()->propagate_.label_ = tmp_xml_ele_->Attribute("name");

  int counter = 0;
  for (auto c_root = tmp_xml_ele_->FirstChildElement("channel");
      c_root != nullptr; c_root = c_root->NextSiblingElement("channel")) {
    const char* type = c_root->Attribute("type");
    if (nullptr == type) {
      LOG_ERROR << "Could not found the 'type' attribute of "
          << c_root->Attribute("name")
          << " in the 'propagates/channel' block";
      continue;
    }

    boost::shared_ptr<Propagate> channel
        = propa_loader_->createInstance<Propagate>(type);
    if (!channel->init(c_root)) {
      LOG_FATAL << "The " << channel->propa_name_ << "channel initialize fail!";
    }
    LOG_INFO << "Push the " << Middleware::instance()->propagate_.size()
        << "st propagates('" << channel->propa_name_
        << "') into '" << Middleware::instance()->propagate_.label_ << "'";

    Middleware::instance()->propagate_.add(channel->propa_name_, channel);
    ++counter;
  }

  tmp_xml_ele_ = nullptr;
  return true;
}

bool Parser::checkHwUnitFormat() {
  if (nullptr == unit_loader_) {
    LOG_FATAL << "unit_loader is nullptr!";
    return false;
  }
  if (Middleware::instance()->propagate_.empty()) {
  // if (nullptr == robot->propagate_.get()) {
    LOG_FATAL << "The instance of Middleware is nullptr"
        << ", Is you call the parserJoints method before parserPropagates method?";
    return false;
  }

  tmp_xml_ele_ = xml_root_->FirstChildElement("hardwares");
  if (nullptr == tmp_xml_ele_) {
    LOG_FATAL << "Could not found the 'hardwares' block.";
    return false;
  }
  if (nullptr == tmp_xml_ele_->Attribute("name")) {
    LOG_WARNING << "The 'hardwares' tag has no 'name' attribute, "
        << "we will use the default name: 'hardwares'";
    tmp_xml_ele_->SetAttribute("name", "hardwares");
  }
  if (nullptr == tmp_xml_ele_->Attribute("list")) {
    LOG_FATAL << "The 'hardwares' tag has no 'list' attribute, "
        << "we will use the default name: 'hardwares'";
    return false;
  }

  return true;
}

/**
 * Parser Helper Method
 * parserJointStates(), parserJoint() and parserPropagates()
 */
bool Parser::parseHwUnits() {

  if (!checkHwUnitFormat()) return false;
  tmp_xml_ele_ = xml_root_->FirstChildElement("hardwares");

  std::vector<std::string> hw_units;
  std::stringstream list_ss;
  list_ss << tmp_xml_ele_->Attribute("list");
  std::string tmp;
  while (list_ss >> tmp) hw_units.push_back(tmp);
  if (hw_units.empty()) {
    LOG_FATAL << "The 'hardwares' tag has no anything!";
    return false;
  }

  for (const auto& unit : hw_units) {
    HwUnitSp group(new HwUnit());
    for (auto jnt_tag = tmp_xml_ele_->FirstChildElement(unit);
          nullptr != jnt_tag; jnt_tag = jnt_tag->NextSiblingElement()) {
      if (nullptr == jnt_tag->Attribute("type")) {
        // joint_handle.reset(new HwUnit(jnt_root->Attribute("name")));
        LOG_FATAL << "No 'type' attribute tag in the '" << unit << "'";
        return false;
      }
      HwUnitSp unit = unit_loader_->createInstance<HwUnit>(jnt_tag->Attribute("type"));
      unit->init(jnt_tag);
      Middleware::instance()->propagate_[unit->cmd_channel_]
                            ->registerHandle(unit->hw_name_, unit->getCmdHandle());
      Middleware::instance()->propagate_[unit->state_channel_]
                            ->registerHandle(unit->hw_name_, unit->getStataHandle());

      group->add(unit->hw_name_, unit);
    }

    Middleware::instance()->hw_unit_.add(group->hw_name_, group);
  }

  return true;
}

} /* namespace qr_driver */
