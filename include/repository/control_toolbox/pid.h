/*
 * pid.h
 *
 *  Created on: Nov 2, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_CONTROL_TOOLBOX_PID_H_
#define INCLUDE_REPOSITORY_CONTROL_TOOLBOX_PID_H_

#include "system/foundation/utf.h"

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

  /*!
   * @brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt.
   *
   * @param error  Error since last call (error = target - state)
   * @param dt     Change in time since last call (in s)
   *
   * @returns PID command
   */
  double computeCommand(double error, double dt);

protected:
  /*!
   * The gain of PID controller read from the configure file under the @prefix
   * tag when the constructor been called. The content of configure as follow:
   * TODO
   */
  class Gains*  gains_;
  ///! This structure saves the variety of errors.
  class Errors* errors_;
  ///! The threshold value.
  double epsilon_;
};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_CONTROL_TOOLBOX_PID_H_ */
