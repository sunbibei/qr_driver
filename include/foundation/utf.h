/*
 * utf.h
 * Uniform Type define File defines all of the typedef or macro.
 *
 *  Created on: Jan 16, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_UTILS_UTF_H_
#define INCLUDE_SYSTEM_UTILS_UTF_H_

#ifndef USING_STD_IO
#include <glog/logging.h>
#include <glog/log_severity.h>
#endif

#include <vector>
#include <string>
#include <map>
// #include <iostream>

// cancel the namespace middleware
// namespace middleware {
#define TIMER_INIT \
    std::chrono::high_resolution_clock::time_point t0; \
    std::chrono::milliseconds sleep_time; \
    t0 = std::chrono::high_resolution_clock::now();

#define TIMER_CONTROL(duration) \
    sleep_time = std::chrono::milliseconds(duration) - std::chrono::duration_cast<std::chrono::milliseconds>( \
        std::chrono::high_resolution_clock::now() - t0); \
    if (sleep_time.count() > 0) { \
      std::this_thread::sleep_for(sleep_time); \
    } \
    t0 = std::chrono::high_resolution_clock::now();

#define SINGLETON_DECLARE(TYPE, ...) \
    protected: \
    TYPE(__VA_ARGS__); \
    virtual ~TYPE(); \
    static TYPE* instance_; \
    public: \
    static TYPE* create_instance(__VA_ARGS__); \
    static TYPE* instance(); \
    static void  destroy_instance(); \
    private:

#define SINGLETON_IMPL(TYPE) \
    TYPE* TYPE::instance_ = nullptr; \
    TYPE* TYPE::create_instance() { \
      if (nullptr != instance_) { \
        LOG_WARNING << "This method 'create_instance()' is called twice."; \
      } else { \
        instance_ = new TYPE(); \
      } \
      return instance_; \
    } \
    TYPE* TYPE::instance() { \
      if (nullptr == instance_) \
        LOG_WARNING << "This method instance() should be called after " \
            << "create_instance()"; \
      return instance_; \
    } \
    void TYPE::destroy_instance() { \
      if (nullptr != instance_) { \
        delete instance_; \
        instance_ = nullptr; \
      } \
    }

#define SINGLETON_IMPL_NO_CREATE(TYPE) \
    TYPE* TYPE::instance_ = nullptr; \
    TYPE* TYPE::instance() { \
      if (nullptr == instance_) \
        LOG_WARNING << "This method instance() should be called after " \
            << "create_instance()"; \
      return instance_; \
    } \
    void TYPE::destroy_instance() { \
      if (nullptr != instance_) { \
        delete instance_; \
        instance_ = nullptr; \
      } \
    }

template<class _Type>
using MiiVector =  std::vector<_Type>;

template<class _Key, class _Value>
using MiiMap    =  std::map<_Key, _Value>;

// typedef std::string MiiString;
using MiiString = std::string;

enum JntDataType {
  UNKNOWN_TYPE = -1,
  POS = 0,
  VEL = 1,
  TOR = 2,
  N_JNT_DATA_TYPES = 3
};

enum LegType {
  UNKNOWN_LEG = -1,
  FL = 0,
  FR = 1,
  HL = 2,
  HR = 3,
  N_LEGS = 4
};

enum JntType {
  UNKNOWN_JNT = -1,
  YAW = 0,
  HIP = 1,
  KNEE = 2,
  N_JNTS = 3
};

#define _DEBUG_INFO_FLAG (true)

#ifndef USING_STD_IO
#define LOG_DEBUG     if (_DEBUG_INFO_FLAG) LOG(WARNING)  << "\t"
#define LOG_INFO      LOG(INFO)     << "\t"
#define LOG_WARNING   LOG(WARNING)  << "\t"
#define LOG_ERROR     LOG(ERROR)    << "\t"
#define LOG_FATAL     LOG(FATAL)    << "\t"
#else
#define LOG_DEBUG     if (_DEBUG_INFO_FLAG) std::cout  << "\t"
#define LOG_INFO      std::cout    << "\t"
#define LOG_WARNING   std::cout    << "\t"
#define LOG_ERROR     std::cout    << "\t"
#define LOG_FATAL     std::cout    << "\t"
#endif
//} /* namespace middleware */



#endif /* INCLUDE_SYSTEM_UTILS_UTF_H_ */
