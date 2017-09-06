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

#include "system/foundation/utf.h"

namespace middleware {

class MiiRobot {
public:
  /**
   * @brief The pure virtual function is asked to implemented by subclass.
   *        These task should be completed in the function what include but not
   *        limited to: instantiate JointManger, PropagateManager, HwManager, and
   *        especially MiiCfgReader. Throwing a fatal exception if something is wrong.
   */
  virtual void create_system_instance();

  virtual bool init();
  virtual bool start();

public:
  /**
   * @brief This methods add the joint command to Joint object.
   * @param _name    The name of controlled joint
   * @param _command The read data of command
   */
  void addCommand(const MiiString& _name, double _command);
  void addCommand(const MiiVector<MiiString>&, const MiiVector<double>&);

  /**
   * @brief This methods add the joint command to Joint object.
   * @param _owner  The owner who owns the specific _jnt want to control
   * @param _jnt    The controlled Joint type
   * @param command The real data of command
   */
  void addCommand(LegType _owner, JntType _jnt, double _command);
  void addCommand(const MiiVector<LegType>&, const MiiVector<JntType>&,
      const MiiVector<double>&);
  /**
   * 获取Joint的名称, 位置, 速度, 力矩及JointState等数据
   * 推荐直接使用获取JointState, 可以一次获取全部数据
   */
  void getJointNames(MiiVector<MiiString>&);
  void getJointPositions(MiiVector<double>&);
  void getJointVelocities(MiiVector<double>&);
  void getJointTorques(MiiVector<double>&);
  /**
   * @brief Instead of the upper methods, you also use the JointManager directly.
   */
  class JointManager* joint_manager()           { return jnt_manager_; }
  const class JointManager& joint_manager_ref() {return *jnt_manager_; }

public:
  static void auto_inst(const MiiString& __p, const MiiString& __type);

protected:
  /**
   * @brief Constructed function.
   * @param __tag Every necessary parameters will be found in this __tag
   */
  MiiRobot(const MiiString& __tag);
  virtual ~MiiRobot();

  /**
   * Given by subclass in the parameters list of the constructed function.
   * Tell MiiRobot what necessary parameters are found in @prefix_tag_.
   */
  MiiString                     prefix_tag_;
  class HwManager*              hw_manager_;
  class JointManager*           jnt_manager_;

  MiiVector<class TouchDown*>          td_list_;
  // type: leg
  MiiVector<class TouchDown*>          td_list_by_type_;
  MiiMap<MiiString, class TouchDown*>  td_list_by_name_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_ */
