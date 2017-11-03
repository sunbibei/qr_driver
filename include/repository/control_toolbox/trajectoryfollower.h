/*
 * qr_driver.h
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#ifndef QUADRUPED_ROBOT_DRIVER_H_
#define QUADRUPED_ROBOT_DRIVER_H_

#include "system/foundation/utf.h"

namespace middleware {

class TrajectoryFollower {
public:
  /**
   * @brief The public methods offer the ability of following joint trajectory.
   *        The trajectory contains time sequence, joint positions for each timestamp,
   *        and joint velocities for each timestamps.
   */
  void stopTraj();
  bool doTraj(const MiiVector<double>& inp_timestamps,
      const MiiVector<MiiVector<double>>& inp_positions,
      const MiiVector<MiiVector<double>>& inp_velocities);

  void pushJointHandle(class Joint*);

protected:
  /**
   * 关节位置/速度控制接口
   */
  void executeJointPositions (const MiiVector<double>&);
  void executeJointVelocities(const MiiVector<double>&);

protected:
  virtual ~TrajectoryFollower();
  // 获取QuadrupedRobotDriver对象实例
  // static Middleware* instance();

private:
  /**
   * @brief The helper method for interpolation，returns positions of the joints at time 't'
   */
  MiiVector<double> interp_cubic(double t, double T,
      const MiiVector<double>& p0_pos, const MiiVector<double>& p1_pos,
      const MiiVector<double>& p0_vel, const MiiVector<double>& p1_vel);

protected:
  TrajectoryFollower();
  MiiVector<class Joint*> jnt_handles_;
  MiiVector<MiiString> jnt_names_;

  // 每次电机指令执行的延时(ms)
  double servoj_time_;
  bool   executing_traj_;
};

} /* namespace quadruped_robot_driver */

#endif /* QUADRUPED_ROBOT_DRIVER_H_ */
