/*
 * propagate.h
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_
#define INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_

#include <vector>
#include <string>
#include <tinyxml.h>

#include "middleware/util/composite.h"
#include "middleware/hardware/hw_unit.h"

namespace middleware {

class Propagate {
public:
  Propagate();
  Propagate(const std::string& name);
  virtual ~Propagate();

  // 本通信方式的名称， 该名称作为Hw_Unit的Channel参数
  std::string propa_name_;

  virtual bool init(TiXmlElement* root = nullptr) {return true;};
  virtual bool write(const std::vector<std::string>&) {return true;};
  virtual bool read() {return true;};
  virtual void stop() { };
  // for Debug
  virtual void check() { ; };
  /**************************************************
   * 下述两个函数注册机构单元的状态或命令句柄
   * 注册句柄, 是为了效率考虑, 注册后, 解析更新后的数据
   * 直接可以在本类中进行更新. 第一个参数为机构单元的名称
   * 应小心使用下述四个函数, 必须传入对应机构单元的状态或命令实际句柄
   * 否则将会发生不能更新数据或断错误
   * 参数1: 指定注册句柄的关节名称
   * 参数2: 指定注册句柄, 共享指针
   * 参数3: 可选, 指定注册的通信通道
   **************************************************/
  void registerHandle(boost::shared_ptr<HwUnit>);

protected:
  Composite<HwCommand>  cmd_composite_;
  Composite<HwState>    state_composite_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_ */
