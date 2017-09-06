/*
 * ThreadManager.h
 *
 *  Created on: Sep 5, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_UTILS_THREADMANAGER_H_
#define INCLUDE_SYSTEM_UTILS_THREADMANAGER_H_

#include "system/foundation/utf.h"

#include <functional>
#include <thread>

namespace middleware {

#define TIMER_INIT \
    std::chrono::high_resolution_clock::time_point t0; \
    std::chrono::milliseconds sleep_time; \
    t0 = std::chrono::high_resolution_clock::now();

#define TIMER_CONTROL(duration) \
    sleep_time = duration - std::chrono::duration_cast<std::chrono::milliseconds>( \
        std::chrono::high_resolution_clock::now() - t0); \
    if (sleep_time.count() > 0) { \
      std::this_thread::sleep_for(sleep_time); \
    } \
    t0 = std::chrono::high_resolution_clock::now();

class ThreadPool {
public:
  static ThreadPool* create_instance();
  static ThreadPool* instance();
  static void        destroy_instance();

protected:
  ThreadPool();
  ~ThreadPool();

public:
  /**
   * @brief This method allow you to add many functions associate with arguments
   *        into the internal vector. This class will start a thread for each
   *        function. An example as follow,
   *        bool debug() { };       void debug1() { }
   *        void print(double) { }; void print(int,int) { }
   *        class SomeClass {
   *          public:
   *          void method() { };
   *          void some_method() { }; void some_method(int) { };
   *        } *obj;
   *
   *        MiiThread t;
   *        t.add(debug); t.add(debug1); t.add(&A::method, obj);
   *        t.add(static_cast<void (&)(double)>(print), 0.123);
   *        t.add(static_cast<void (&)(int,int)>(print), 10, 100);
   *        t.add(static_cast<void (SomeClass::*)()>(&A::some_method), obj);
   *        t.add(static_cast<void (SomeClass::*)(int)>(&A::some_method). obj, 100);
   * @param __n        The name of thread
   * @param __interval The interval of thread's loop, it uses the default time(20ms)
   *                   if the given value less than zero.
   * @param __f        The address of function
   * @param __args     The variadic templates offer the list of arguments.
   */
  template<typename _Func, typename... _BoundArgs>
  void add(const MiiString& __n, _Func&& __f, _BoundArgs&&... __args);

  bool init();
  bool start();
  bool start(const MiiString& __n);
  bool start(const MiiVector<MiiString>& __n);
  void stop();
  void stop(const MiiString& __n);
  void stop(const MiiVector<MiiString>& __n);

  bool is_running(const MiiString& __n);

protected:
  static ThreadPool*             instance_;
  /**
   * The list of thread function and variate.
   */
  MiiMap<MiiString, std::function<void()>>      thread_funcs_;
  MiiMap<MiiString, class __PrivateThreadVar*>  thread_vars_;
};




///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
template<typename _Func, typename... _BoundArgs>
void ThreadPool::add(const MiiString& __n, _Func&& __f, _BoundArgs&&... __args) {
  if (thread_funcs_.end() != thread_funcs_.find(__n))
    LOG_WARNING << "The named thread(" << __n << ") task function "
      << "has inserted into the function list. It will be replaced.";
  thread_funcs_.insert(std::make_pair(__n, std::bind(__f, __args...)));
}

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_UTILS_THREADMANAGER_H_ */
