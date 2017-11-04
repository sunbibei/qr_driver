/*
 * pid.cpp
 *
 *  Created on: Nov 2, 2017
 *      Author: bibei
 */

#include <repository/control_toolbox/pid.h>
#include <system/foundation/cfg_reader.h>

#include <cmath>

namespace middleware {

const double INVALID_TARGET = 0xFFFFFFFFl;

inline const double& __clamp(const double& a, const double& b, const double& c) {
    return std::min(std::max(b, a), c);
}

struct Gains {
  // Optional constructor for passing in values
  Gains(double p, double i, double d)
    : p_gain_(p),
      i_gain_(i),
      d_gain_(d)
  {}
  // Default constructor
  Gains()
    : p_gain_(0.0),
      i_gain_(0.0),
      d_gain_(0.0)
  {}
  ///! Proportional gain.
  double p_gain_;
  ///! Integral gain.
  double i_gain_;
  ///! Derivative gain.
  double d_gain_;
};

struct Errors {
  Errors()
  : p_error_last_(0), p_error_(0), i_error_(0), d_error_(0),
    i_max_(0), i_min_(0), antiwindup_(false), tmp_d_error_(0) { }
  ///! Save position state for derivative state calculation.
  double p_error_last_;
  ///! Position error.
  double p_error_;
  ///! Integral of position error.
  double i_error_;
  ///! Derivative of position error.
  double d_error_;
  ///! Maximum allowable integral error.
  double i_max_;
  ///! Minimum allowable integral error.
  double i_min_;
  ///! Antiwindup
  bool   antiwindup_;
  ///! The update interface, e = target - state
  bool update(double e, double dt) {
    // Calculate the derivative error
    tmp_d_error_  = (e - p_error_last_) / dt;
    if (std::isnan(tmp_d_error_) || std::isinf(tmp_d_error_))
      return false;
    p_error_      = e;
    d_error_      = tmp_d_error_;
    p_error_last_ = p_error_;
    // Calculate the integral of the position error
    i_error_ += dt * p_error_;
    if (antiwindup_) i_error_ = __clamp(i_error_, i_min_, i_max_);
    return true;
  }

  void clear() {
    p_error_last_ = 0;
    p_error_      = 0;
    i_error_      = 0;
    d_error_      = 0;
  }

private:
  double tmp_d_error_;
};

inline double operator*(const Gains& k, const Errors& e) {
  return k.p_gain_ * e.p_error_
       + k.i_gain_ * e.i_error_
       + k.d_gain_ * e.d_error_;
}

inline double operator*(const Errors& e, const Gains& k) {
  return k * e;
}

Pid::Pid(const MiiString& prefix)
  : gains_(new Gains), errors_(new Errors),
    target_(INVALID_TARGET), epsilon_(10),
    first_compute_(true), dt_(0.0) {
  auto cfg = MiiCfgReader::instance();

  MiiVector<double> tmp_vec;
  cfg->get_value_fatal(prefix, "gains", tmp_vec);
  if (3 == tmp_vec.size()) {
    gains_->p_gain_ = tmp_vec[0];
    gains_->i_gain_ = tmp_vec[1];
    gains_->d_gain_ = tmp_vec[2];
  }

  tmp_vec.clear();
  cfg->get_value(prefix, "limits", tmp_vec);
  if (3 == tmp_vec.size()) {
    errors_->i_min_ = tmp_vec[0];
    errors_->i_max_ = tmp_vec[1];
    epsilon_        = tmp_vec[2];
  }

  cfg->get_value(prefix, "antiwindup", errors_->antiwindup_);
}

Pid::~Pid() {
  if (nullptr != gains_) {
    delete gains_;
    gains_ = nullptr;
  }
  if (nullptr != errors_) {
    delete errors_;
    errors_ = nullptr;
  }
}

void Pid::setTarget(double target) {
  target_ = target;
}

bool Pid::compute(short _x, short& _u) {
  if (INVALID_TARGET == target_ || std::isnan(_x) || std::isinf(_x))
    return false;
  if (std::abs(target_ - _x) < epsilon_) {
    errors_->clear();
    target_ = INVALID_TARGET;
    _u = 0.0;
    return true;
  }

  curr_update_t_ = std::chrono::high_resolution_clock::now();
  dt_ = ((first_compute_) ? (0) : (std::chrono::duration_cast<std::chrono::nanoseconds>(
      curr_update_t_ - last_update_t_).count() / std::nano::den));
  first_compute_ = false;
  // Update the variety of error
  _u =  (errors_->update(target_ - _x, dt_)) ?
        ((*gains_) * (*errors_)) : 0.0;
  last_update_t_ = curr_update_t_;

  if (true)
    printf("%1.20f %1.20f %1.20f %1.20f %1.20f %1.20f %1.20f",
        gains_->p_gain_, gains_->i_gain_, gains_->d_gain_,
        errors_->p_error_, errors_->i_error_, errors_->d_error_, _u);

  return true;
}

} /* namespace middleware */
