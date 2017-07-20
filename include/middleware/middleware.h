/*
 * qr_driver.h
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#ifndef QUADRUPED_ROBOT_DRIVER_H_
#define QUADRUPED_ROBOT_DRIVER_H_

#include <mutex>
#include <thread>
#include <boost/shared_ptr.hpp>

#include "middleware/propagate/propagate.h"
#include "middleware/hardware/hw_unit.h"
#include "middleware/util/composite.h"
#include "middleware/util/noncopyable.h"

namespace middleware {

typedef boost::shared_ptr<HwCommand> HwCmdSp;
typedef boost::shared_ptr<HwState>   HwStateSp;
typedef boost::shared_ptr<Propagate> PropaSp;
typedef boost::shared_ptr<HwUnit>    HwUnitSp;

class Middleware : public NonCopyable {
public:
  // Unit and Propagate interface
  Composite<Propagate>  propagate_;
  HwUnit                hw_unit_;

  std::vector<std::string> jnt_names_;

  // 初始化所有变量, 以及线程等.
#ifndef ROS_BUILD
  bool init(const std::string& xml);
#endif

  bool init();

  // 开始/停止运行
  bool start();
  void halt();

  /**
   * 设定关节命令, 并发送给机器人
   * 参数1: 指定关节名称
   * 参数2: 指定命令数据
   */
  void addCommand(const std::string&, const HwCommand&);
  void addCommand(const std::string&, const HwCmdSp&);
  void addCommand(const std::vector<std::string>&, const std::vector<HwCmdSp>&);
  void addCommand(const std::vector<std::string>&, const std::vector<HwCommand>&);
  /**
   * 获取Joint的名称, 位置, 速度, 力矩及JointState等数据
   * 推荐直接使用获取JointState, 可以一次获取全部数据
   */
  void getJointNames(std::vector<std::string>&);
  void getJointPositions(std::vector<double>&);
  void getJointVelocities(std::vector<double>&);
  void getJointTorques(std::vector<double>&);
  // void getJointStates(sensor_msgs::JointState&);

  /**
   * 执行/停止轨迹命令执行
   */
  void stopTraj();
  bool doTraj(const std::vector<double>& inp_timestamps,
      const std::vector<std::vector<double>>& inp_positions,
      const std::vector<std::vector<double>>& inp_velocities);

  /**
   * 关节位置/速度控制接口
   */
  void executeJointPositions(const std::vector<std::string>&, const std::vector<double>&);
  void executeJointVelocities(const std::vector<std::string>&, const std::vector<double>&);
public:
  ~Middleware();
  // 获取QuadrupedRobotDriver对象实例
  static Middleware* instance();

private:
  /**
   * 插值函数， Returns positions of the joints at time 't'
   */
  std::vector<double> interp_cubic(double t, double T,
      const std::vector<double>& p0_pos, const std::vector<double>& p1_pos,
      const std::vector<double>& p0_vel, const std::vector<double>& p1_vel);

private:
  Middleware();

  /*
   * 创建硬件接口的工厂方法
   * 从robot.xml文件中读取配置信息， 完成propagate_和robot_的初始化。
   */
  static Middleware* instance_;

  void runPropagate();

  // 用于保存每一次需要写入命令的关节名称列表
  // 设定命令和写命令使用互斥锁
  std::mutex cmd_lock_;
  bool new_command_;
  std::vector<std::string> new_jnt_cmd_names_;

  // 每次电机指令执行的延时(ms)
  double servoj_time_;
  bool executing_traj_;
  bool keepalive_;
  bool connected_;
  std::thread* propagate_thread_;
};

} /* namespace quadruped_robot_driver */

#endif /* QUADRUPED_ROBOT_DRIVER_H_ */
