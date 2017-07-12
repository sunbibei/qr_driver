/*
 * robot_parser.h
 *
 *  Created on: Dec 3, 2016
 *      Author: silence
 */

#ifndef INCLUDE_ROBOT_PARSER_H_
#define INCLUDE_ROBOT_PARSER_H_

#include <map>
#include <string>
#include <tinyxml.h>
#include <ros/ros.h>
#include <class_loader/class_loader.h>

namespace middleware {

class Middleware;
/**
 * 从参数文件中解析初始化middleware对象.
 * 该类作为工具类, 没必要进行实例化或继承
 */
class Parser final {
public:
  /**
   * API, 从文件或ROS 参数中解析对象
   * 两个函数的实现, 仅与初始化方式不同而已
   */
  static bool parse(const std::string&);
  static bool parse();

private:
  static TiXmlElement* xml_root_;
  static class_loader::ClassLoader* propa_loader_;
  static class_loader::ClassLoader* unit_loader_;

  static TiXmlElement* tmp_xml_ele_;

  // 初始化函数, 从文件或ROS参数
  static bool init(const std::string&);
  static bool init();
  // initHelper
  static bool initVariants(TiXmlDocument*);

  // 解析器Helper, 分别解析'joint_states' TAG and 'propagates' TAG
  static bool parsePropagates();
  static bool parseHwUnit();

  inline static bool checkPropagatesFormat();
  inline static bool checkHwUnitFormat();
};

} /* namespace middleware */

#endif /* INCLUDE_ROBOT_PARSER_H_ */
