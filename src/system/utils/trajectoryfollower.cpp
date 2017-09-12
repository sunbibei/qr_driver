/*
 * qr_driver.cpp
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */


#include <repository/resource/joint.h>
#include <system/foundation/label.h>
#include <system/utils/trajectoryfollower.h>

#include <thread>

namespace middleware {

TrajectoryFollower::TrajectoryFollower()
    : servoj_time_(200), executing_traj_(false) {
}

TrajectoryFollower::~TrajectoryFollower() {
}

void TrajectoryFollower::pushJointHandle(Joint* jnt) {
  jnt_handles_.push_back(jnt);
}

void TrajectoryFollower::stopTraj() {
  executing_traj_ = false;
}

bool TrajectoryFollower::doTraj(const MiiVector<double>& inp_timestamps,
      const MiiVector<MiiVector<double>>& inp_positions,
      const MiiVector<MiiVector<double>>& inp_velocities) {
  std::chrono::high_resolution_clock::time_point t0, t;
  std::vector<double> positions;
  unsigned int j;
  executing_traj_ = true;
  t0 = std::chrono::high_resolution_clock::now();
  t = t0;
  j = 0;
  while ((inp_timestamps[inp_timestamps.size() - 1]
      >= std::chrono::duration_cast<std::chrono::duration<double>>(t - t0).count())
      and executing_traj_) { // 确保仍在轨迹时间内
    while (inp_timestamps[j]
        <= std::chrono::duration_cast<std::chrono::duration<double>>(
            t - t0).count() && j < inp_timestamps.size() - 1) {
      j += 1; // 跳转到距离当前执行时间最近的一个轨迹点
    }
    // 计算得到当前时间所要达到的关节位置
    positions = interp_cubic(
        // 实际执行该轨迹点的时间与该轨迹点的前一个点要求时间的差值
        std::chrono::duration_cast<std::chrono::duration<double>>
            (t - t0).count() - inp_timestamps[j - 1],
        // 待执行轨迹点与前一个点理论时间间距
        inp_timestamps[j] - inp_timestamps[j - 1],
        // 待执行轨迹点与前一个点的要求位置
        inp_positions[j - 1], inp_positions[j],
        // 待执行轨迹点与前一个点的要求速度
        inp_velocities[j - 1], inp_velocities[j]);
    // TODO 是否需要新增名称参数?
    executeJointPositions(positions);

    // oversample with 4 * sample_time
    std::this_thread::sleep_for(
        std::chrono::milliseconds((int)(servoj_time_ / 4.)));
    t = std::chrono::high_resolution_clock::now();
  }
  executing_traj_ = false;
  return true;
}

inline void TrajectoryFollower::executeJointPositions(const MiiVector<double>& _pos) {
  if (jnt_handles_.size() != _pos.size()) return;

  for (size_t i = 0; i < _pos.size(); ++i)
    jnt_handles_[i]->updateJointCommand(_pos[i], JntCmdType::POS);
}

void TrajectoryFollower::executeJointVelocities(const MiiVector<double>& _vels) {
  if (jnt_handles_.size() != _vels.size()) return;

  for (size_t i = 0; i < _vels.size(); ++i)
    jnt_handles_[i]->updateJointCommand(_vels[i], JntCmdType::VEL);

}

/**
 * 计算得到t时刻的关节位置
 * @param t       实际执行时间
 * @param T       执行总时长
 * @param p0_pos  起始点位置
 * @param p1_pos  终止点位置
 * @param p0_vel  起始点速度
 * @param p1_vel  终止点速度
 */
std::vector<double> TrajectoryFollower::interp_cubic(double t, double T,
    const std::vector<double>& p0_pos, const std::vector<double>& p1_pos,
    const std::vector<double>& p0_vel, const std::vector<double>& p1_vel) {
  /* Returns positions of the joints at time 't' */
  std::vector<double> positions;
  for (unsigned int i = 0; i < p0_pos.size(); i++) {
    double a = p0_pos[i];
    double b = p0_vel[i];
    double c = (-3 * p0_pos[i] + 3 * p1_pos[i] - 2 * T * p0_vel[i]
        - T * p1_vel[i]) / pow(T, 2);
    double d = (2 * p0_pos[i] - 2 * p1_pos[i] + T * p0_vel[i]
        + T * p1_vel[i]) / pow(T, 3);
    positions.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
  }
  return positions;
}

} /* namespace quadruped_robot_driver */
