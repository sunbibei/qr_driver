/*
 * hw_manager.cpp
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#include <boost/bind.hpp>
#include <system/platform/hw_manager.h>
#include <system/platform/propagate/propagate_manager.h>
#include <system/foundation/utf.h>

namespace middleware {

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

const size_t MAX_PKTS_SIZE = 512;
HwManager* HwManager::instance_ = nullptr;

HwManager* HwManager::create_instance() {
  if (nullptr != instance_)
    LOG_WARNING << "This method 'HwManager::create_instance()' is called twice.";
  else
    instance_ = new HwManager();

  return instance_;
}

void HwManager::destroy_instance() {
  if (nullptr != instance_) {
    delete instance_;
    instance_ = nullptr;
  }
}

HwManager* HwManager::instance() {
  if (nullptr == instance_)
    LOG_WARNING << "This method HwManager::instance() should be called after "
        << "HwManager::create_instance()";

  return instance_;
}

HwManager::HwManager()
: tick_interval_(10), thread_alive_(true), tick_thread_(nullptr),
  propagate_manager_(nullptr) {
  packets_.reserve(MAX_PKTS_SIZE);
}

HwManager::~HwManager() {
  thread_alive_ = false;
  if (nullptr != tick_thread_) {
    tick_thread_->join();
    delete tick_thread_;
    tick_thread_ = nullptr;
  }
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
  LOG_INFO << "NAME\tADDR\tID\tPUB\tCMD";
  for (auto hw : res_list_) {
    memset(debug_info, '\0', 1024 * sizeof(char));
    sprintf(debug_info, "%s\t%d\t%d\t%d",hw->getLabel().c_str(), hw,
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
  if (nullptr != tick_thread_) {
    LOG_WARNING << "Call HwManager::run() twice!";
    return false;
  }

  tick_thread_ = new std::thread(
            boost::bind(&HwManager::tick, this));
  LOG_INFO << "The propagate thread which for the real-time message communication "
      << "has started.";
  return true;
}

void HwManager::tick() {

  TIME_INIT
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

    TIME_CONTROL(tick_interval_)
  }
}

} /* namespace middleware */
