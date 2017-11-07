/*
 * pid.cpp
 *
 *  Created on: Nov 2, 2017
 *      Author: bibei
 */

#include <repository/control_toolbox/pid.h>
#include <system/foundation/cfg_reader.h>

#include <boost/algorithm/clamp.hpp>
#include <fstream>
#include <cmath>

namespace middleware {

const short INVALID_TARGET = 0x8888;

// Output file description for debug
const size_t BUF_RESERVE_SIZE = 1024;
bool                   __g_debug;
std::ofstream          __g_ofd;
std::vector<short>     __g_u_buf;
std::vector<short>     __g_x_buf;
std::vector<double>    __g_t_buf;

inline void __save_everything_to_file() {
  if ((!__g_debug) || !__g_ofd.is_open()) return;
  for (size_t i = 0; i < __g_t_buf.size(); ++i)
    __g_ofd << __g_t_buf[i] << " " << __g_x_buf[i] << " " << __g_u_buf[i] << std::endl;
  __g_ofd << std::endl;

  LOG_DEBUG << "It has saved everything into the local file.";
  __g_u_buf.clear();
  __g_x_buf.clear();
  __g_t_buf.clear();
}

inline void __save_data_into_buf(double dt, short _x, short _u) {
  if ((!__g_debug) || !__g_ofd.is_open()) return;
  __g_t_buf << dt;
  __g_x_buf << _x;
  __g_u_buf << _u;
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
    if (!dt) return false;
    // Calculate the derivative error
    tmp_d_error_  = (e - p_error_last_) / dt;
    if (std::isnan(tmp_d_error_) || std::isinf(tmp_d_error_))
      return false;
    p_error_      = e;
    d_error_      = tmp_d_error_;
    p_error_last_ = p_error_;
    // Calculate the integral of the position error
    i_error_ += dt * p_error_;
    if (antiwindup_) i_error_ = boost::algorithm::clamp(i_error_, i_min_, i_max_);
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
  if (5 == tmp_vec.size()) {
    errors_->i_min_ = tmp_vec[0];
    errors_->i_max_ = tmp_vec[1];
    cmd_min_        = tmp_vec[2];
    cmd_max_        = tmp_vec[3];
    epsilon_        = tmp_vec[4];
  }

  cfg->get_value(prefix, "antiwindup", errors_->antiwindup_);
  MiiString tmp;
  Label::split_label(prefix, tmp, name_);

  __g_debug = false;
  if (cfg->get_value(prefix, "debug", __g_debug) && __g_debug) {
    MiiString path = ".";
    cfg->get_value(prefix, "path", path);
    __g_ofd.open(path + "/" + name_
        + "_" + std::to_string(gains_->p_gain_)
        + "_" + std::to_string(gains_->i_gain_)
        + "_" + std::to_string(gains_->d_gain_));

    __g_x_buf.reserve(BUF_RESERVE_SIZE);
    __g_u_buf.reserve(BUF_RESERVE_SIZE);
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

void Pid::setTarget(short target) {
  target_ = target;
  t0_     = std::chrono::high_resolution_clock::now();
  errors_->clear();
  first_compute_ = true;
}

bool Pid::control(short _x, short& _u) {
  if (INVALID_TARGET == target_ || std::isnan(_x) || std::isinf(_x))
      return false;
  // The system has not went to stabilization.
  if (!stability(_x, _u)) {
    // compute the command
    if (compute(_x, _u))
      // safety control
      safety_control(_x, _u);
    else
      return false;
  }
  __save_data_into_buf(dt_, _x, _u);
  return true;
}

// TODO
bool Pid::stability(short _x, short& _u) {
  return (std::abs(target_ - _x) <= epsilon_);
}

// TODO
bool Pid::compute(short _x, short& _u) {
  // if (0 != name_.compare("pid_0")) return false;

  if (std::abs(target_ - _x) <= epsilon_) {
    /*if (!first_compute_) {
      t1_ = std::chrono::high_resolution_clock::now();
      printf("%s - elapse(ms): %d error: %d\n", name_.c_str(),
        std::chrono::duration_cast<std::chrono::milliseconds>(
        t1_ - t0_).count(), target_ - _x);
      // t0_ = t1_;
    }*/

    first_compute_ = true;
    // target_        = INVALID_TARGET;
    _u             = 0.0;
    //printf("%s - %04d %04d %04d\n",
    //     name_.c_str(), target_, _x, target_ - _x);
    return true;
  }
 
  curr_update_t_ = std::chrono::high_resolution_clock::now();
  dt_ = ((first_compute_) ? (0) : (std::chrono::duration_cast<std::chrono::milliseconds>(
      curr_update_t_ - last_update_t_).count()/* / std::nano::den*/));
  // dt_ /= 1000.0;
  first_compute_ = false;
  // Update the variety of error
  ///! dt (in ms) convert to in s.
  _u =  (errors_->update(target_ - _x, dt_/1000.0)) ?
        ((*gains_) * (*errors_)) : 0.0;
  _u =  boost::algorithm::clamp(_u, cmd_min_, cmd_max_);
  last_update_t_ = curr_update_t_;

  if (_u && false)
    printf("%s - %01.3f %4.2f %4.2f %1.2f %04d %04d %04d\n",
        name_.c_str(), dt_,
        errors_->p_error_, errors_->i_error_, errors_->d_error_,
        target_, _x, _u);

  return true;
}

// TODO
void Pid::safety_control(short _x, short& _u) {
  return;
}

} /* namespace middleware */
