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

  /**
   * 等待子类实现的函数s
   */
  virtual bool init(TiXmlElement* root);
  virtual bool start() { return true; }
  /**
   * TODO
   */
  virtual bool write(void*, size_t, uint32_t) {return true;};
  /**
   * TODO
   */
  virtual int  read(void*, size_t, uint32_t&) {return 0;};
  virtual void stop() { };
  // for Debug
  virtual void check() {
    LOG_WARNING << "=============CHECK=============";
    LOG_INFO << "COMMAND:\nNAME\tADDR\tCOUNT";
    for (const auto& c : cmd_composite_) {
      LOG_INFO << c.first << "\t" << c.second.get() << "\t" << c.second.use_count();
    }
    LOG_INFO << "STATE:\nNAME\tADDR\tCOUNT";
    for (const auto& s : state_composite_) {
      LOG_INFO << s.first << "\t" << s.second.get() << "\t" << s.second.use_count();
    }
    LOG_WARNING << "===============================";
  }
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
  void registerHandle(const std::string&, HwStateSp);
  void registerHandle(const std::string&, HwCmdSp);
  // void registerHandle(HwUnit*);
  bool send(const std::vector<std::string>&);
  bool recv();
  // void changeCacheSize(int new_size);
  // void parse();
  // void parse(const std::vector<std::string>&);
protected:
  size_t                propa_r_cache_size_;
  size_t                propa_w_cache_size_;
  size_t                cache_r_offset_;
  size_t                cache_w_offset_;
  size_t                cache_w_base_;
  uint8_t*              propa_r_cache_;
  uint8_t*              propa_w_cache_;
  uint32_t              recv_index_;

  bool                  tmp_ret_;
  int                   tmp_read_size_;

  Composite<HwCommand>  cmd_composite_;
  Composite<HwState>    state_composite_;

private:
  class Command*  proto_cmd_;
  class Feedback* proto_fb_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_ */
