/*
 * hw_manager.cpp
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#include <system/platform/hw_manager.h>
#include <system/platform/propagate/propagate_manager.h>
#include <system/foundation/utf.h>
#include "system/platform/thread/threadpool.h"

namespace middleware {

#define HW_MANAGER_THREAD ("hw_manager_thread")
const size_t MAX_PKTS_SIZE = 512;

SINGLETON_IMPL(HwManager)

HwManager::HwManager()
: ResourceManager<HwUnit>(), tick_interval_(10), thread_alive_(false),
  propagate_manager_(PropagateManager::create_instance()) {
  packets_.reserve(MAX_PKTS_SIZE);
}

HwManager::~HwManager() {
  thread_alive_ = false;
  ThreadPool::instance()->stop(HW_MANAGER_THREAD);
}

bool HwManager::init() {
  if (res_list_.empty()) {
    LOG_WARNING << "There are no any hardware push to the HwManager";
    return false;
  }

  LOG_INFO << "Start to order the hw_unit, in total " << res_list_.size();
  // We have any idea about the maxium id, so we use the maxium value about unsigned char.
  hw_list_by_id_.resize((unsigned char)0xFF);
  hw_list_by_cmd_.reserve(res_list_.size());

  char* debug_info = new char[1024];
  LOG_INFO << "========================================";
  LOG_INFO << "NAME\t\tADDR\t\tNODE_ID\tCMD";
  for (auto hw : res_list_) {
    memset(debug_info, '\0', 1024 * sizeof(char));
    sprintf(debug_info, "%s\t0x%02X\t0x%02X\t%d",hw->getLabel().c_str(), hw,
        hw->node_id_, hw->requireCmdDeliver());
    LOG_INFO << debug_info;

    hw_list_by_id_[hw->node_id_] = hw;
    hw_list_by_name_.insert(std::make_pair(hw->getLabel().c_str(), hw));
    if (hw->requireCmdDeliver()) {
      hw_list_by_cmd_.push_back(hw);
    }
  }

  delete[] debug_info;
  debug_info = nullptr;
  LOG_INFO << "========================================";

  return true;
}

bool HwManager::run() {
  if (ThreadPool::instance()->is_running(HW_MANAGER_THREAD)) {
    LOG_WARNING << "Call HwManager::run() twice!";
    return false;
  }

  LOG_DEBUG << "==========HwManager::run==========";
  if (!propagate_manager_->run()) {
    LOG_WARNING << "PropagateManager::run fail!";
  }
  LOG_INFO << "Starting PropagateManager";

  ThreadPool::instance()->add(HW_MANAGER_THREAD, &HwManager::tick, this);
  LOG_DEBUG << "==========HwManager::run==========";
  return true;
}

void HwManager::tick() {
  for (auto& hw : res_list_) {
    hw->init();
  }


  TIMER_INIT
  while (thread_alive_) {
    packets_.clear();
    propagate_manager_->readPackets(packets_);
    // The manager delivers each packet which read from Propagate for hardware update.
    for (const auto& pkt : packets_) {
      hw_list_by_id_[pkt.node_id]->handleMsg(pkt);
    }

    // Collecting all of the new command to control the robot
    packets_.clear();
    for (auto hw : hw_list_by_cmd_) {
      hw->generateCmd(packets_);
        // packets_.push_back(pkt);
    }
    propagate_manager_->writePackets(packets_);

    TIMER_CONTROL(tick_interval_)
  }
}

} /* namespace middleware */
