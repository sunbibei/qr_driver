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

const short INVALID_VALUE = 0x8888;

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
  __g_t_buf .push_back(dt);
  __g_x_buf .push_back(_x);
  __g_u_buf .push_back(_u);
}

struct Gains {
  // Optional constructor for passing in values
  Gains(double p, double i, double d)
    : p_gain_(p), i_gain_(i), d_gain_(d),
      i_max_(0), i_min_(0)
  {}
  // Default constructor
  Gains()
    : p_gain_(0.0), i_gain_(0.0), d_gain_(0.0),
      i_max_(0), i_min_(0)
  {}
  ///! Proportional gain.
  double p_gain_;
  ///! Integral gain.
  double i_gain_;
  ///! Derivative gain.
  double d_gain_;
  ///! Maximum allowable integral iterm.
  double i_max_;
  ///! Minimum allowable integral iterm.
  double i_min_;
};

struct Errors {
  Errors()
  : p_error_last_(0), p_error_(0), i_error_(0), d_error_(0),
    tmp_d_error_(0) { }
  ///! Save position state for derivative state calculation.
  double p_error_last_;
  ///! Position error.
  double p_error_;
  ///! Integral of position error.
  double i_error_;
  ///! Derivative of position error.
  double d_error_;
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
    i_error_     += dt * p_error_;
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
       + boost::algorithm::clamp(k.i_gain_ * e.i_error_, k.i_min_, k.i_max_)
       + k.d_gain_ * e.d_error_;
}

inline double operator*(const Errors& e, const Gains& k) {
  return k * e;
}

struct Limits {
  enum Type {
    MIN = 0,
    MAX,
    N_TYPE
  };
  short command[Type::N_TYPE];
  short angle[Type::N_TYPE];
};

struct TimeControl {
  ///! the first compute
  bool  first_compute_;
  ///! time control (in ms)
  int64_t dt_;
  std::chrono::high_resolution_clock::time_point curr_update_t_;
  std::chrono::high_resolution_clock::time_point last_update_t_;

  ///! these variables for debug.
  std::chrono::high_resolution_clock::time_point t0_;
  std::chrono::high_resolution_clock::time_point t1_;

  bool running() {
    return (last_update_t_ != INVALID_TIME_POINT);
  }

  void start() {
    curr_update_t_ = std::chrono::high_resolution_clock::now();
    last_update_t_ = curr_update_t_;
    t0_            = curr_update_t_;
  }

  /*!
   * @brief The duration (in ms)
   */
  int64_t dt() {
    curr_update_t_ = std::chrono::high_resolution_clock::now();
    dt_            = std::chrono::duration_cast<std::chrono::milliseconds>
        (curr_update_t_ - last_update_t_).count();
    last_update_t_ = curr_update_t_;
    return dt_;
  }

  void stop() {
    t1_ = std::chrono::high_resolution_clock::now();
    curr_update_t_ = INVALID_TIME_POINT;
    last_update_t_ = INVALID_TIME_POINT;
  }

private:
  const std::chrono::high_resolution_clock::time_point INVALID_TIME_POINT
    = std::chrono::high_resolution_clock::time_point::max();
};

class Stability {
private:
  ///! The stability margin;
  short  epsilon_;
  ///! The times waiting.
  size_t lapse_;
  ///! The target value
  short target_;
  bool  internal_flag_;
  ///! The stability array
  MiiVector<short> x_buf_;
  MiiVector<short> e_buf_;

  const size_t BUF_REV_SIZE = 64;

public:
  Stability(short epsilon, size_t lapse)
    : epsilon_(epsilon), lapse_(lapse),
      target_(INVALID_VALUE),
      internal_flag_(false) {
    x_buf_.reserve(BUF_REV_SIZE);
    e_buf_.reserve(BUF_REV_SIZE);
  }

  /*!
   * @brief Get the target
   */
  short target() { return target_; }

  /*!
   * @brief This method set a new target.
   */
  void set_target(short target) {
    internal_flag_ = false;
    x_buf_.clear();
    e_buf_.clear();
    target_ = target;
  }

  /*!
   * @brief This method judges the current state whether is stable or not,
   *        return true if the system is stability, or return false.
   *        @_e is error.
   */
  bool classifier(short _x, short& _e) {
    _e = (INVALID_VALUE == target_) ? (INVALID_VALUE) : (target_ - _x);
    if (x_buf_.size() >= BUF_REV_SIZE) {
      x_buf_.clear();
      e_buf_.clear();
    }

    if (internal_flag_) {
      // mean filter.
      _x += (x_buf_.empty()) ? 0 : x_buf_.back();
      x_buf_.push_back(_x);
      _x /= x_buf_.size();
      // re-calculate the error.
      _e = target_ - _x;
      e_buf_.push_back(_e + ((e_buf_.empty()) ? 0 : e_buf_.back()));
    } else {
      internal_flag_ = (std::abs(_e) <= epsilon_);
    }

    return ((e_buf_.size() >= lapse_)
        && (std::abs(e_buf_.back()/e_buf_.size()) <= epsilon_));
  }

};

Pid::Pid(const MiiString& prefix)
  : gains_(new Gains), errors_(new Errors),
    limits_(new Limits), time_control_(new TimeControl),
    stability_(nullptr) {
  auto cfg = MiiCfgReader::instance();

  MiiVector<double> tmp_vec_d;
  cfg->get_value_fatal(prefix, "gains", tmp_vec_d);
  if (5 == tmp_vec_d.size()) {
    gains_->p_gain_ = tmp_vec_d[0];
    gains_->i_gain_ = tmp_vec_d[1];
    gains_->d_gain_ = tmp_vec_d[2];
    gains_->i_min_  = tmp_vec_d[3];
    gains_->i_max_  = tmp_vec_d[4];
  }

  MiiVector<short> tmp_vec_s;
  cfg->get_value(prefix, "limits", tmp_vec_s);
  if (4 == tmp_vec_s.size()) {
    limits_->command[Limits::MIN] = tmp_vec_s[0];
    limits_->command[Limits::MAX] = tmp_vec_s[1];
    limits_->angle[Limits::MIN]   = tmp_vec_s[2];
    limits_->angle[Limits::MAX]   = tmp_vec_s[3];
  }

  tmp_vec_s.clear();
  cfg->get_value(prefix, "criterion", tmp_vec_s);
  if (2 == tmp_vec_s.size())
    stability_ = new Stability(tmp_vec_s[0], tmp_vec_s[1]);

  MiiString tmp;
  Label::split_label(prefix, tmp, name_);

  // For debug function
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
  stability_->set_target(target);
  errors_->clear();
  time_control_->start();
}

bool Pid::control(short _x, short& _u) {
  if (!time_control_->running() || std::isnan(_x) || std::isinf(_x))
      return false;

  short error = 0;
  // The system has not went to stabilization.
  if (!stability(_x, error) && INVALID_VALUE != error)
    // compute the command
    compute(_x, error, _u);
  else
    _u = 0;

  // safety control
  safety_control(_x, _u);

  __save_data_into_buf(time_control_->dt_, _x, _u);
  return true;
}

bool Pid::stability(short _x, short& _e) {
  return stability_->classifier(_x, _e);
}

void Pid::compute(short _x, short _e, short& _u) {
  _u =  (errors_->update(_e, time_control_->dt()/1000.0)) ?
        ((*gains_) * (*errors_)) : 0.0;

  if (_u && false)
    printf("%s - %ld %4.2f %4.2f %1.2f %04d %04d %04d\n",
        name_.c_str(), time_control_->dt_,
        errors_->p_error_, errors_->i_error_, errors_->d_error_,
        stability_->target(), _x, _u);
}

void Pid::safety_control(short _x, short& _u) {
  _u = boost::algorithm::clamp(_u, limits_->command[Limits::MIN], limits_->command[Limits::MAX]);
  if (_x <= limits_->angle[Limits::MIN] || _x >= limits_->angle[Limits::MAX]) _u = 0;
}

} /* namespace middleware */
