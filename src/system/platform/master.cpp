/*
 * hw_manager.cpp
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#include <system/platform/propagate/propagate_manager.h>
#include <system/platform/sw_node/sw_node_manager.h>
#include <system/foundation/utf.h>
#include <system/platform/master.h>
#include "system/platform/thread/threadpool.h"

namespace middleware {

#define MASTER_THREAD ("master")
const size_t MAX_PKTS_SIZE = 512;

SINGLETON_IMPL(Master)

Master::Master()
: tick_interval_(10), thread_alive_(false),
  propagate_manager_(PropagateManager::create_instance()),
  sw_node_manager_(SWNodeManager::create_instance()) {
  packets_.reserve(MAX_PKTS_SIZE);
}

Master::~Master() {
  thread_alive_ = false;
  PropagateManager::destroy_instance();
  SWNodeManager::destroy_instance();
  propagate_manager_ = nullptr;
  sw_node_manager_   = nullptr;
}

bool Master::init() {
  if (!propagate_manager_ || !sw_node_manager_) return false;

  return sw_node_manager_->init();
}

bool Master::run() {
  if (ThreadPool::instance()->is_running(MASTER_THREAD)) {
    LOG_WARNING << "Call HwManager::run() twice!";
    return false;
  }

  LOG_DEBUG << "==========Master::run==========";
  if (!propagate_manager_->run()) {
    LOG_WARNING << "PropagateManager::run fail!";
  }
  LOG_INFO << "Starting PropagateManager";

  thread_alive_ = true;
  ThreadPool::instance()->add(MASTER_THREAD, &Master::tick, this);
  LOG_DEBUG << "==========Master::run==========";
  return true;
}

void Master::tick() {
  TIMER_INIT

  while (thread_alive_) {

    // The manager delivers each packet which read from Propagate for hardware update.
    packets_.clear();
    propagate_manager_->readPackets(packets_);
    sw_node_manager_->handleMsg(packets_);

    // Just for debug
    for (const Packet& pkt : packets_) {
      if (false)
        printf("NODE_ID: 0x%02X, MSG_ID: 0x%02X, LEN: %d, \
          DATA: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          pkt.node_id, pkt.msg_id, pkt.size, pkt.data[0],
          pkt.data[1], pkt.data[2], pkt.data[3], pkt.data[4],
          pkt.data[5], pkt.data[6], pkt.data[7]);
      // hw_list_by_id_[pkt.node_id]->handleMsg(pkt);
    }

    // Collecting all of the new command to control the robot
    packets_.clear();
    sw_node_manager_->generateCmd(packets_);
    if (!packets_.empty()) LOG_DEBUG << "Got Command from SWNode, size=" << packets_.size();
    else LOG_DEBUG << "No Command from SWNode";
    propagate_manager_->writePackets(packets_);

    TIMER_CONTROL(tick_interval_)
  }
}

} /* namespace middleware */
