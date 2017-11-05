/*
 * pid.h
 *
 *  Created on: Nov 2, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_CONTROL_TOOLBOX_PID_H_
#define INCLUDE_REPOSITORY_CONTROL_TOOLBOX_PID_H_

#include "system/foundation/utf.h"
#include <chrono>

namespace middleware {

class Pid {
public:
  Pid(const MiiString& prefix);
  virtual ~Pid();

public:
  /*!
   * @brief This method offer the interface to change the gains in the runtime.
   * @param Kp    The proportional gain.
   * @param Ki    The integral gain.
   * @param Kd    The derivative gain.
   */
  void setPid(double Kp, double Ki, double Kd);

  void setTarget(short);

  /*!
   * @brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt.
   *
   * @param _x  State since last call
   * @param _u  Command, range from -5000 to 5000
   *
   * @returns PID command
   */
  bool compute(short _x, short& _u);

protected:
  /*!
   * The gain of PID controller read from the configure file under the @prefix
   * tag when the constructor been called. The content of configure as follow:
   * <pid_0  gains="Kp Ki Kd" limits = "iMin iMax error_threshold" />
   * <pid_0  node_id="0x02" leg="fl" jnt="yaw"  gains="1.20 0.10 0.10" limits="0 200 10" />
   */
  class Gains*  gains_;
  ///! This structure saves the variety of errors.
  class Errors* errors_;
  ///! The minum of command
  short cmd_min_;
  ///! The maxum of command
  short cmd_max_;
  ///! The target value
  short target_;
  ///! The threshold value.
  short epsilon_;
  ///! the first compute
  bool  first_compute_;

  ///! time control
  std::chrono::high_resolution_clock::time_point curr_update_t_;
  std::chrono::high_resolution_clock::time_point last_update_t_;
  double dt_;

  ///!
  MiiString name_;

  std::chrono::high_resolution_clock::time_point t0_;
  std::chrono::high_resolution_clock::time_point t1_;
};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_CONTROL_TOOLBOX_PID_H_ */
