/*
 * sw_node_manager.cpp
 *
 *  Created on: Sep 8, 2017
 *      Author: bibei
 */

#include <system/platform/sw_node/sw_node_manager.h>
#include <iomanip>

namespace middleware {

SINGLETON_IMPL(SWNodeManager)

SWNodeManager::SWNodeManager()
  : internal::ResourceManager<SWNode>() {

}

SWNodeManager::~SWNodeManager() {
}

bool SWNodeManager::init() {
  if (res_list_.empty()) {
    LOG_WARNING << "There are no any hardware push to the HwManager";
    return false;
  }

  LOG_INFO << "Start to order the hw_unit, in total " << res_list_.size();
  // We have any idea about the maxium id, so we use the maxium value about unsigned char.
  hw_list_by_id_.resize((unsigned char)0xFF);
  hw_list_by_cmd_.reserve(res_list_.size());

  char* debug_info = new char[1024];
  LOG_INFO << "=====================================================";
  LOG_INFO << "NAME\t\tADDR\t\tNODE_ID\tCMD";
  // unsigned char max_node_id = 0x00;
  for (auto hw : res_list_) {
    //memset(debug_info, '\0', 1024 * sizeof(char));
    //sprintf(debug_info, "%s\t0x%02X\t0x%02X\t%d",hw->getLabel().c_str(), hw,
    //    hw->node_id_, hw->requireCmdDeliver());
    //LOG_INFO << debug_info;
    LOG_INFO << hw->getLabel() << "\t" << hw << "\t0x"
        << std::setw(2) << std::setfill('0') << std::hex << (int)hw->node_id_
        << "\t" << (hw->requireCmdDeliver()?"YES":"NO");

    hw_list_by_id_[hw->node_id_] = hw;
    hw_list_by_name_.insert(std::make_pair(hw->getLabel().c_str(), hw));
    if (hw->requireCmdDeliver()) {
      hw_list_by_cmd_.push_back(hw);
    }
  }

  delete[] debug_info;
  debug_info = nullptr;
  LOG_INFO << "=====================================================";

  return true;
}

void SWNodeManager::handleMsg(const MiiVector<Packet>& pkts) {
  for (const auto& pkt : pkts) {
    if ((hw_list_by_id_.size() <= pkt.node_id)
      || (nullptr == hw_list_by_id_[pkt.node_id])) {
      LOG_ERROR << "What fucking message!(" << (int)pkt.node_id << "/"
        << hw_list_by_id_.size() << "), Address: 0x" << hw_list_by_id_[pkt.node_id];
      continue;
    }
    hw_list_by_id_[pkt.node_id]->handleMsg(pkt);
  }
}

void SWNodeManager::generateCmd(MiiVector<Packet>& pkts) {
  for (auto& node : hw_list_by_cmd_) {
    if (node->generateCmd(pkts))
      ; //LOG_DEBUG << "SW Node: " << node->getLabel() << " generate the command.";
  }
}

} /* namespace middleware */
