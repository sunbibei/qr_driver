/*
 * mii_robot.h
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_
#define INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_

#include <map>
#include <vector>

#include "system/label/label.h"

namespace middleware {

class MiiRobot {
public:
  /**
   * @brief The pure virtual function is asked to implemented by subclass
   *        The function should be completed these tasks include but not
   *        limited to: instantiate JointManger, PropagateManager, HwManager,
   *        MiiCfgReader etc. Throw a fatal exception if something is wrong.
   */
  virtual void create_system_instance() = 0;
  /**
   * 子类在实现本函数时，必须完成初始化MiiCfgReader对象
   */
  virtual bool init();
  virtual bool start();

public:
  /**
   * @brief This methods add the joint command to Joint object.
   * @param _name    The name of controlled joint
   * @param _command The read data of command
   */
  void addCommand(const std::string& _name, double _command);
  void addCommand(const std::vector<std::string>&, const std::vector<double>&);

  /**
   * @brief This methods add the joint command to Joint object.
   * @param _owner  The owner who owns the specific _jnt want to control
   * @param _jnt    The controlled Joint type
   * @param command The real data of command
   */
  void addCommand(LegType _owner, JntType _jnt, double _command);
  void addCommand(const std::vector<LegType>&, const std::vector<JntType>&,
      const std::vector<double>&);
  /**
   * 获取Joint的名称, 位置, 速度, 力矩及JointState等数据
   * 推荐直接使用获取JointState, 可以一次获取全部数据
   */
  void getJointNames(std::vector<std::string>&);
  void getJointPositions(std::vector<double>&);
  void getJointVelocities(std::vector<double>&);
  void getJointTorques(std::vector<double>&);

public:
  static void auto_inst(MiiStringConstRef __p, MiiStringConstRef __type);

protected:
  /**
   * @brief Constructed function.
   * @param __tag Every necessary parameters will be found in this __tag
   */
  MiiRobot(MiiStringConstRef __tag);
  virtual ~MiiRobot();

  /**
   * Given by subclass in the parameters list of the constructed function.
   * Tell MiiRobot what necessary parameters are found in @prefix_tag_.
   */
  MiiString                     prefix_tag_;
  class HwManager*              hw_manager_;
  class JointManager*           jnt_manager_;

  std::vector<class TouchDown*>          td_list_;
  // type: leg
  std::vector<class TouchDown*>          td_list_by_type_;
  std::map<MiiString, class TouchDown*>  td_list_by_name_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_ */
