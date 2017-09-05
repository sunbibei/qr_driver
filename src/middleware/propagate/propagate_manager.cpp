/*
 * propagate_manager.cpp
 *
 *  Created on: Aug 29, 2017
 *      Author: bibei
 */

#include "middleware/propagate/propagate_manager.h"
#include "system/thread/threadpool.h"

#include <boost/bind.hpp>

namespace middleware {

#define MUTEX_TRY_LOCK(locker)    while (!locker.try_lock()) { };
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

const MiiString THREAD_NAME = "propagate";
const size_t MAX_QUEUE_SIZE = 1024;
PropagateManager* PropagateManager::instance_ = nullptr;

PropagateManager* PropagateManager::instance() {
  if (nullptr == instance_) {
    instance_ = new PropagateManager();
  }

  return instance_;
}

PropagateManager::PropagateManager()
  : propa_interval_(20), thread_alive_(true) {
  pkts_queue_4_send_.reserve(MAX_QUEUE_SIZE);
  pkts_queue_4_recv_.reserve(MAX_QUEUE_SIZE);
}

PropagateManager::~PropagateManager() {
  thread_alive_ = false;
  ThreadPool::instance()->stop(THREAD_NAME);

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
  if (ThreadPool::instance()->is_running(THREAD_NAME)) {
    LOG_WARNING << "Call PropagateManager::run() twice!";
    return false;
  }

  for (auto& c : res_list_) {
    if (!c->start())
      LOG_WARNING << "The propagate '" << c->propa_name_ << "' starting fail.";
    else
      LOG_INFO << "The propagate '" << c->propa_name_ << "' has started.";
  }

  ThreadPool::instance()->add(THREAD_NAME, &PropagateManager::updatePktsQueues, this);

  /*propa_thread_ = new std::thread(
            boost::bind(&PropagateManager::updatePktsQueues, this));
  LOG_INFO << "The propagate thread which for the real-time message communication "
      << "has started.";*/
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

bool PropagateManager::readPackets(std::vector<Packet>& pkts) {
  MUTEX_TRY_LOCK(lock_4_recv_)
  if (!pkts_queue_4_recv_.empty())
    for (const auto& pkt : pkts_queue_4_recv_)
      pkts.push_back(pkt);
  MUTEX_UNLOCK(lock_4_recv_)
  return true;
}

bool PropagateManager::writePackets(const std::vector<Packet>& pkts) {
  MUTEX_TRY_LOCK(lock_4_send_)
  if (!pkts.empty())
    for (const auto& pkt : pkts)
      pkts_queue_4_send_.push_back(pkt);
  MUTEX_UNLOCK(lock_4_send_)
  return true;
}

} /* namespace middleware */
