/*
 * thread.cpp
 *
 *  Created on: Sep 5, 2017
 *      Author: bibei
 */

#include <boost/bind.hpp>
#include <system/thread/threadpool.h>
#include <chrono>
#include <thread>

namespace middleware {

struct __PrivateThreadVar {
  bool         thread_started_;
  std::thread* thread_handle_;
  __PrivateThreadVar() : thread_started_(false), thread_handle_(nullptr) { }
};

ThreadPool* ThreadPool::instance_ = nullptr;
ThreadPool* ThreadPool::create_instance() {
  if (nullptr != instance_) {
    LOG_WARNING << "This method 'ThreadManager::create_instance()' is called twice.";
  } else {
    instance_ = new ThreadPool;
  }

  return instance_;
}

ThreadPool* ThreadPool::instance() {
  if (nullptr == instance_)
    LOG_WARNING << "This method ThreadManager::instance() should be called after "
        << "ThreadManager::create_instance()";

  return instance_;
}

void ThreadPool::destroy_instance() {
  if (nullptr != instance_) {
    delete instance_;
    instance_ = nullptr;
  }
}


ThreadPool::ThreadPool() {
}

ThreadPool::~ThreadPool() {
  stop();
}

bool ThreadPool::init() {
  return true;
}

bool ThreadPool::start() {
  bool ret = true;
  for (auto& f : thread_funcs_) {
    ret  = ret && start(f.first);
  }
  return ret;
}

bool ThreadPool::start(const MiiString& __n) {
  // start the specific named threads
  auto t = thread_funcs_.find(__n);
  if (thread_funcs_.end() == t) {
    LOG_WARNING << "The task function of the specific named thread "
        << "isn't exist in function list.";
    return false;
  }

  auto var = thread_vars_.find(__n);
  if (thread_vars_.end() == var) {
    thread_vars_.insert(std::make_pair(__n, new __PrivateThreadVar));
    thread_vars_[__n]->thread_handle_  = new std::thread(t->second);
    thread_vars_[__n]->thread_started_ = true;
    LOG_INFO << "This thread(" << __n << ") has started.";
  } else {
    LOG_WARNING << "This thread(" << __n << ") has started ago.";
  }

  return true;
}

bool ThreadPool::start(const MiiVector<MiiString>& __ns) {
  bool ret = true;
  for (const auto& __n : __ns)
    ret = ret && start(__n);

  return ret;
}

void ThreadPool::stop() {
  for (auto& var : thread_vars_)
    stop(var.first);

  thread_vars_.clear();
}

void ThreadPool::stop(const MiiString& __n) {
  auto var = thread_vars_.find(__n);
  if (thread_vars_.end() == var) {
    LOG_WARNING << "These is not exist the named thread(" << __n
        << ") in the thread pool.";
    return;
  }

  if (nullptr != var->second) {
    if (nullptr != var->second->thread_handle_) {
      var->second->thread_handle_->join();
      delete var->second->thread_handle_;
      var->second->thread_handle_ = nullptr;
    }
    delete var->second;
    var->second = nullptr;
  }

  thread_vars_.erase(var);
}

void ThreadPool::stop(const MiiVector<MiiString>& __ns) {
  for (const auto& __n : __ns)
    stop(__n);
}

bool ThreadPool::is_running(const MiiString& __n) {
  auto var = thread_vars_.find(__n);
  if (thread_vars_.end() == var) {
    LOG_WARNING << "These is not exist the named thread(" << __n
        << ") in the thread pool.";
    return false;
  }

  return true;
}

} /* namespace middleware */
