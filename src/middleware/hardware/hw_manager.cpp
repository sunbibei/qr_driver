/*
 * hw_manager.cpp
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#include <middleware/hardware/hw_manager.h>
#include <system/utils/log.h>

#include "middleware/hardware/hw_unit.h"

namespace middleware {

HwManager* HwManager::instance_ = nullptr;

/*HwManager::~HwManager() {
  if (nullptr != s_propagate_manager_) {
    delete s_propagate_manager_;
    s_propagate_manager_ = nullptr;
  }

  for (auto& hw : s_hw_list_) {
    delete hw;
    hw = nullptr;
  }

  LOG_INFO << "Destroy all of the hw_unit";
}*/

HwManager* HwManager::instance() {
  if (nullptr == instance_) instance_ = new HwManager();

  return instance_;
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
    sprintf(debug_info, "%s\t%02X\t%d\t%d",hw->hw_name_, hw,
        hw->node_id_, hw->requireCmdDeliver());
    LOG_INFO << debug_info;

    hw_list_by_id_[hw->node_id_] = hw;
    hw_list_by_name_.insert(std::make_pair(hw->hw_name_, hw));
    if (hw->requireCmdDeliver()) {
      hw_list_by_cmd_.push_back(hw);
    }
  }

  delete[] debug_info;
  debug_info = nullptr;
  LOG_INFO << "========================================";

  return true;
}

void HwManager::tick() {
  packets_.clear();
  /*TODO get Packet from robot*/
  // The manager delivers each packet which read from Propagate for hardware update.
  for (const auto& pkt : packets_) {
    hw_list_by_id_[pkt.node_id]->handleMsg(pkt);
  }

  // Collecting all of the new command to control the robot
  packets_.clear();
  for (auto hw : hw_list_by_cmd_) {
    Packet pkt;
    if (hw->generateCmd(pkt))
      packets_.push_back(pkt);
    // TODO write the Packet to robot
  }


}

} /* namespace middleware */
