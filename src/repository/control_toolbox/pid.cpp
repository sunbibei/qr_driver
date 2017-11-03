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
    i_max_(0), i_min_(0), clamp_(false), tmp_d_error_(0) { }
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
  ///!
  bool   clamp_;
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
    if (clamp_) i_error_ = __clamp(i_error_, i_min_, i_max_);
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
  : gains_(new Gains), errors_(new Errors), epsilon_(0.0) {
  auto cfg = MiiCfgReader::instance();

  MiiVector<double> tmp_vec;
  cfg->get_value(prefix, "gains", tmp_vec);
  if (3 == tmp_vec.size()) {
    gains_->p_gain_ = tmp_vec[0];
    gains_->i_gain_ = tmp_vec[1];
    gains_->d_gain_ = tmp_vec[2];
  }

  tmp_vec.clear();
  errors_->clamp_ = cfg->get_value(prefix, "limits", tmp_vec);
  if (3 == tmp_vec.size()) {
    errors_->i_min_ = tmp_vec[0];
    errors_->i_max_ = tmp_vec[1];
    epsilon_        = tmp_vec[2];
  }
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

double Pid::computeCommand(double error, double dt) {
  if (dt <= 0.0 || std::isnan(error) || std::isinf(error))
    return 0.0;
  if (std::abs(error) < epsilon_) {
    errors_->clear();
    return 0.0;
  }
  // Update the variety of error
  if (!errors_->update(error, dt)) return 0.0;

  return (*gains_) * (*errors_);
}

} /* namespace middleware */
