/*
 * propagate_manager.cpp
 *
 *  Created on: Aug 29, 2017
 *      Author: bibei
 */

#include "middleware/propagate/propagate_manager.h"
#include <boost/bind.hpp>

namespace middleware {

#define MUTEX_TRY_LOCK(locker)    while (!locker.try_lock()) { }
#define MUTEX_UNLOCK(locker)  locker.unlock();
#define SEND_EVERY_PTKS     \
    while (!pkts_queue_4_send_.empty()) { \
      const auto& pkt = pkts_queue_4_send_.back(); \
      for (auto& c : res_list_) { \
        if (c->write(pkt)) break;\
      } \
      pkts_queue_4_send_.pop_back(); \
    }

#define RECV_EVERY_PTKS \
    Packet pkt; \
    for (auto& c : res_list_) { \
      if (c->read(pkt)) pkts_queue_4_recv_.push_back(pkt); \
    }

#define TIME_INIT \
    std::chrono::high_resolution_clock::time_point t0; \
    std::chrono::milliseconds sleep_time; \
    t0 = std::chrono::high_resolution_clock::now();

#define TIME_CONTROL(duration) \
    sleep_time = duration - std::chrono::duration_cast<std::chrono::milliseconds>( \
        std::chrono::high_resolution_clock::now() - t0); \
    if (sleep_time.count() > 0) { \
      std::this_thread::sleep_for(sleep_time); \
    } \
    t0 = std::chrono::high_resolution_clock::now();

const size_t MAX_QUEUE_SIZE = 1024;
PropagateManager* PropagateManager::instance_ = nullptr;

PropagateManager* PropagateManager::instance() {
  if (nullptr == instance_) {
    instance_ = new PropagateManager();
  }

  return instance_;
}

PropagateManager::PropagateManager()
  : propa_interval_(20), thread_alive_(true), propa_thread_(nullptr) {
  pkts_queue_4_send_.reserve(MAX_QUEUE_SIZE);
  pkts_queue_4_recv_.reserve(MAX_QUEUE_SIZE);
}

PropagateManager::~PropagateManager() {
  thread_alive_ = false;
  if (nullptr != propa_thread_) {
    propa_thread_->join();
    delete propa_thread_;
    propa_thread_ = nullptr;
  }

  /*if (!pkts_queue_4_send_.empty()) {
    for (auto& pkt : pkts_queue_4_send_) {
      delete pkt;
      pkt = nullptr;
    }
  }

  if (!pkts_queue_4_recv_.empty()) {
    for (auto& pkt : pkts_queue_4_recv_) {
      delete pkt;
      pkt = nullptr;
    }
  }*/
}

bool PropagateManager::run() {
  if (nullptr != propa_thread_) {
    LOG_WARNING << "Call PropagateManager::run() twice!";
    return false;
  }

  for (auto& c : res_list_) {
    if (!c->start())
      LOG_WARNING << "The propagate '" << c->propa_name_ << "' starting fail.";
    else
      LOG_INFO << "The propagate '" << c->propa_name_ << "' has started.";
  }
  propa_thread_ = new std::thread(
            boost::bind(&PropagateManager::updatePktsQueues, this));
  LOG_INFO << "The propagate thread which for the real-time message communication "
      << "has started.";
  return true;
}

void PropagateManager::updatePktsQueues() {
  TIME_INIT
  while (thread_alive_) {
    MUTEX_TRY_LOCK(lock_4_send_)
    SEND_EVERY_PTKS
    MUTEX_UNLOCK(lock_4_send_)

    MUTEX_TRY_LOCK(lock_4_recv_)
    RECV_EVERY_PTKS
    MUTEX_UNLOCK(lock_4_recv_)

    TIME_CONTROL(propa_interval_)
  }
}

} /* namespace middleware */
