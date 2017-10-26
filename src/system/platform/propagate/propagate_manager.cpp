/*
 * propagate_manager.cpp
 *
 *  Created on: Aug 29, 2017
 *      Author: bibei
 */

#include "system/platform/propagate/propagate_manager.h"

#include "system/platform/thread/threadpool.h"

namespace middleware {

#define MUTEX_TRY_LOCK(locker)    while (!locker.try_lock()) { };
#define MUTEX_UNLOCK(locker)  locker.unlock();

const MiiString THREAD_NAME = "propagate";
const size_t MAX_QUEUE_SIZE = 1024;
std::thread update_thread_;

SINGLETON_IMPL(PropagateManager)

PropagateManager::PropagateManager()
  : internal::ResourceManager<Propagate>(),
    propa_interval_(1), thread_alive_(true) {

  pkts_queue_4_send_.reserve(MAX_QUEUE_SIZE);
  pkts_queue_4_recv_.reserve(MAX_QUEUE_SIZE);
}

PropagateManager::~PropagateManager() {
  thread_alive_ = false;
  ThreadPool::instance()->stop(THREAD_NAME);
  for (auto& c : res_list_) {
    c->stop();
  }
}

bool PropagateManager::run() {
  if (ThreadPool::instance()->is_running(THREAD_NAME)) {
    LOG_WARNING << "Call PropagateManager::run() twice!";
    return false;
  }
  LOG_DEBUG << "==========PropagateManager::run==========";

  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "The list of Propagate: ";
    LOG_WARNING << "=====================================================";
    for (size_t i = 0; i < res_list_.size(); ++i)
      LOG_INFO << i + 1 << ": " << res_list_[i]->propa_name_ << "\t"
          << res_list_[i]->getLabel() << "\t" << res_list_[i];
    LOG_WARNING << "=====================================================";
  }
  for (auto c : res_list_) {
    // for (auto c = begin(); c != end(); ++c) {
    LOG_DEBUG << c->getLabel() << " is starting.";
    if (!c->start())
      LOG_ERROR << "The propagate '" << c->propa_name_ << "' starting FAIL.";
    else
      LOG_DEBUG << "The propagate '" << c->propa_name_ << "' has started.";
  }

  ThreadPool::instance()->add(THREAD_NAME, &PropagateManager::update, this);
  // update_thread_ = std::thread(std::bind(&PropagateManager::update, this));
  return true;
}

void PropagateManager::update() {
  TIMER_INIT

  while (thread_alive_) {
    MUTEX_TRY_LOCK(lock_4_send_)
    while (!pkts_queue_4_send_.empty()) {
      const auto& pkt = pkts_queue_4_send_.back();
      for (auto& c : res_list_) {
        if (c->write(pkt)) break;
      }
      pkts_queue_4_send_.pop_back();
    }
    MUTEX_UNLOCK(lock_4_send_)

    MUTEX_TRY_LOCK(lock_4_recv_)
    for (auto& c : res_list_) {
      Packet pkt;
      if (c->read(pkt)) pkts_queue_4_recv_.push_back(pkt);
    }
    MUTEX_UNLOCK(lock_4_recv_)

    TIMER_CONTROL(propa_interval_)
  }
}

bool PropagateManager::readPackets(std::vector<Packet>& pkts) {
  MUTEX_TRY_LOCK(lock_4_recv_)
  if (!pkts_queue_4_recv_.empty()) {
    for (const auto& pkt : pkts_queue_4_recv_)
      pkts.push_back(pkt);

    pkts_queue_4_recv_.clear();
  }
  MUTEX_UNLOCK(lock_4_recv_)
  return true;
}

bool PropagateManager::writePackets(const std::vector<Packet>& pkts) {
  MUTEX_TRY_LOCK(lock_4_send_)
  if (!pkts.empty()) {
    for (const auto& pkt : pkts)
      pkts_queue_4_send_.push_back(pkt);
  }
  MUTEX_UNLOCK(lock_4_send_)
  return true;
}

} /* namespace middleware */
