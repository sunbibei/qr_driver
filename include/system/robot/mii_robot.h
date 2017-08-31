/*
 * mii_robot.h
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_
#define INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_

#include "system/label/label.h"

#include <map>
#include <vector>

namespace middleware {

class MiiRobot : public Label {
public:
  /**
   * 子类在实现本函数时，必须完成初始化MiiCfgReader对象
   */
  virtual bool init();
  virtual bool start();

public:
  /**
   * 设定关节命令, 并发送给机器人
   * 参数1: 指定关节名称
   * 参数2: 指定命令数据
   */
  void addCommand(const std::string&, double command);
  void addCommand(const std::vector<std::string>&, const std::vector<double>&);
  /**
   * 获取Joint的名称, 位置, 速度, 力矩及JointState等数据
   * 推荐直接使用获取JointState, 可以一次获取全部数据
   */
  void getJointNames(std::vector<std::string>&);
  void getJointPositions(std::vector<double>&);
  void getJointVelocities(std::vector<double>&);
  void getJointTorques(std::vector<double>&);

public:
  static void auto_inst(const MiiString&, const MiiString&);

protected:
  MiiRobot(const MiiString&);
  virtual ~MiiRobot();

  class HwManager*              hw_manager_;
  std::vector<class Joint*>     joint_list_;
  std::vector<class TouchDown*> td_list_;

  std::map<MiiString, class Joint*>     joint_list_by_name_;
  std::map<MiiString, class TouchDown*> td_list_by_name_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_ */
