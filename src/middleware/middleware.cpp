/*
 * qr_driver.cpp
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#include <system/utils/parser.h>
#include <thread>
#include <tinyxml.h>

#include "middleware/middleware.h"
#include "middleware/hardware/joint.h"

namespace middleware {

Middleware* Middleware::instance_ = nullptr;
Middleware* Middleware::instance() {
  if (nullptr == instance_) {
    instance_ = new Middleware;
    LOG_INFO << "Create and return the singleton instance: Middleware";
    return instance_;
  }

  LOG_INFO << "Return the singleton instance: Middleware";
  return instance_;
}

Middleware::Middleware()
    : new_command_(false), servoj_time_(200),
      executing_traj_(false), keepalive_(true),
      connected_(false), propagate_thread_(nullptr) {
}

Middleware::~Middleware() {
  this->halt();
  /*propagate_->clear();
  hw_unit_->clear();*/
  /*if (nullptr != instance_) {
    delete instance_;
    instance_ = nullptr;
  }*/
}

#ifndef ROS_BUILD
bool Middleware::init(const std::string& xml) {
  if (!Parser::parse(xml)) {
    LOG_ERROR << "The initialization FAIL in the Middleware";
    return false;
  }

  LOG_INFO << "The initialization has successful";
  return true;
}
#endif

bool Middleware::init() {
  if (!Parser::parse()) {
    LOG_FATAL << "The initialization FATAL FAIL in the Middleware";
    return false;
  }

  LOG_INFO << "The initialization has successful";
  return true;
}

bool Middleware::start() {
  propagate_thread_ = new std::thread(&Middleware::runPropagate, this);
  LOG_INFO << "The propagate thread has started to run!";
  return true;
}

/*void Middleware::runPropagate() {
  while (keepalive_) {
    while (connected_ && keepalive_) {
      // Everything is OK!
      for (auto& p : propagate_)
        connected_ &= p.second->recv();
      if (new_command_) {
        while (!cmd_lock_.try_lock()) {}
        for (auto& p : propagate_)
          connected_ &= p.second->send(new_jnt_cmd_names_);

        new_command_ = false;
        new_jnt_cmd_names_.clear();
        cmd_lock_.unlock();
      }
    }
    if (keepalive_) {
      // reconnect
      LOG_WARNING << "Disconnected! In order to keep alive, we try to reconnect... ...";
      int count = 0;
      while (keepalive_ && !connected_) {
        propagate_.stop();
        LOG_WARNING << "Attempt to reconnect (" << count++ << " times)";
        connected_ = propagate_.start();
        if (!connected_) {
          // wait for 500ms
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      } // end while(keepalive_ && !connected_)
    } // end if (keepalive_)
  } // end while (keepalive_)
  propagate_.stop();
}*/

void Middleware::halt() {
  keepalive_ = false;
  if (nullptr != propagate_thread_) {
    propagate_thread_->join();
    delete propagate_thread_;
    propagate_thread_ = nullptr;
  }

  LOG_INFO << "Middleware halt... ...";
}

void Middleware::stopTraj() {
  executing_traj_ = false;
}

bool Middleware::doTraj(const std::vector<double>& inp_timestamps,
      const std::vector<std::vector<double>>& inp_positions,
      const std::vector<std::vector<double>>& inp_velocities) {
  std::chrono::high_resolution_clock::time_point t0, t;
  std::vector<double> positions;
  unsigned int j;

  if ((propagate_.empty()) || (hw_unit_.empty())) {
    LOG_ERROR << "Invalidate propagate or robot state";
    return false;
  }
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
    executeJointPositions(jnt_names_, positions);

    // oversample with 4 * sample_time
    std::this_thread::sleep_for(
        std::chrono::milliseconds((int)(servoj_time_ / 4.)));
    t = std::chrono::high_resolution_clock::now();
  }
  executing_traj_ = false;
  return true;
}

void Middleware::executeJointPositions(const std::vector<std::string>& names, const std::vector<double>& positions) {
  ; // TODO
  std::vector<HwCmdSp> cmd_vec;
  cmd_vec.reserve(jnt_names_.size());

  // TODO 需要实际公式计算， 当前实现版本仅仅是电机的位置控制
  // 并未转换到Joint速度指令
  for (std::size_t i = 0; i < jnt_names_.size(); ++i) {
    auto cmd = boost::dynamic_pointer_cast<Joint::CmdType>(hw_unit_[names[i]]->getCommand());
    cmd->command_ = positions[i];
    cmd->mode_    = JntCmdType::POS;
    cmd_vec.push_back(cmd);
  }
  addCommand(names, cmd_vec);
}

void Middleware::executeJointVelocities(const std::vector<std::string>& names, const std::vector<double>& velocities) {
  // TODO do some rate limiting?
  std::vector<HwCmdSp> cmd_vec;
  cmd_vec.reserve(jnt_names_.size());

  // TODO 需要实际公式计算， 当前实现版本仅仅是电机的速度控制
  // 并未转换到Joint速度指令
  for (std::size_t i = 0; i < jnt_names_.size(); ++i) {
    auto cmd = boost::dynamic_pointer_cast<Joint::CmdType>(hw_unit_[names[i]]->getCommand());
    cmd->command_ = velocities[i];
    cmd->mode_    = JntCmdType::VEL;
    cmd_vec.push_back(cmd);
  }
  addCommand(names, cmd_vec);
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
std::vector<double> Middleware::interp_cubic(double t, double T,
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
