/*
 * robot_state_base.h
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#ifndef ROBOT_STATE_BASE_H_
#define ROBOT_STATE_BASE_H_

#include <map>
#include <string>
#include <tinyxml.h>
#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>

#include "middleware/propagate/proto/dragon.pb.h"

namespace middleware {

/**
 * 状态基类
 */
struct HwState {
  HwState() { };
  virtual ~HwState() { };

  virtual bool update(const Feedback*) { return true; };
};

/**
 * 命令基类
 */
struct HwCommand {
  HwCommand() { };
  virtual ~HwCommand() { };

  virtual bool update(Command*) { return true; };
};

class HwUnit {
public:
  /**************************************************
   * 任何该类的子类, 若包含State or Cmd
   * 都必须重定义下述四个typedef
   **************************************************/
  typedef HwState                       StateType;
  typedef HwCommand                     CmdType;
  typedef boost::shared_ptr<HwState>    StateTypeSp;
  typedef boost::shared_ptr<HwCommand>  CmdTypeSp;

  HwUnit();
  HwUnit(const std::string& name);
  virtual ~HwUnit();

  // for Debug
  virtual void check();
  /**
   * 初始化本类对象, 使用xml文件中内容
   * 基类默认实现为空
   */
  virtual bool init(TiXmlElement*);
  /**
   * 子类必须实现下述虚函数
   * 返回所保存状态/命令数据的地址
   * 用以在初始化时注册到Propagate中
   */
  virtual StateTypeSp   getStataHandle();
  virtual CmdTypeSp     getCmdHandle();
/**************************************************
 * 下述四个函数选择性进行实现, 在函数内部, 需要完成数据的读写.
 * 若HwUnit子类具备State or Command
 * 则必须重写对应的get/set函数实现
 * get函数, 以名称为标识符, get对应名称的对象,
 * 为了防止使用后不手动释放内存, 强制使用智能指针
 * set函数, 也以名称为标识符, 设定对应的数据
 **************************************************/
  virtual StateTypeSp getState();
  virtual CmdTypeSp getCommand();
  virtual void setState(const StateType&);
  virtual void setCommand(const CmdType&);

  void setName(const std::string& n) {hw_name_ = n;};
  const std::string& getName() {return hw_name_;};

/*  virtual void setStateChannel(const std::string& c) {state_channel_ = c;};
  virtual void setCmdChannel(const std::string& c) {cmd_channel_ = c;};
  virtual const std::string& getStateChannel() {return state_channel_;};
  virtual const std::string& getCmdChannel() {return cmd_channel_;};

protected:*/
  std::string hw_name_;
  std::string state_channel_;
  std::string cmd_channel_;
};

typedef boost::shared_ptr<HwState>   HwStateSp;
typedef boost::shared_ptr<HwCommand> HwCmdSp;

} /* namespace quadruped_robot_driver */

#endif /* ROBOT_STATE_BASE_H_ */
