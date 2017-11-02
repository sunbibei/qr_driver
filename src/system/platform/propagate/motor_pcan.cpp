/*
 * motor_pcan.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: bibei
 */

#include <system/platform/propagate/motor_pcan.h>

namespace middleware {

MotorPcan::MotorPcan(const MiiString& l)
  : ArmPcan(l) {
  ;
}

bool MotorPcan::init() {
  /*if (!Propagate::init()) return false;

  auto& cfg = MiiCfgReader::instance();

  unsigned char c = PCAN_USBBUS1;
  cfg->get_value(getLabel(), "channel",   c);
  pcan_config_.channel = c;

  cfg->get_value(getLabel(), "baud_rate", pcan_config_.baud_rate);
  cfg->get_value(getLabel(), "type",      pcan_config_.type);
  cfg->get_value(getLabel(), "port",      pcan_config_.port);
  cfg->get_value(getLabel(), "interrupt", pcan_config_.interrupt);*/
  return true;
}

MotorPcan::~MotorPcan() {
  ;
}

bool MotorPcan::write(const Packet& pkt) {
  if (MII_MSG_COMMON_DATA_1 == pkt.msg_id) {
    int offset = 0;
    for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
      if ((INVALID_BYTE == pkt.data[offset]) && (INVALID_BYTE == pkt.data[offset + 1]))
        continue;

      memcpy(&(T_[pkt.node_id][type]), pkt.data + offset, sizeof(short));
      offset += sizeof(short);
    }
  }

  return ArmPcan::write(pkt);
}

bool MotorPcan::read(Packet& pkt) {
  if (!ArmPcan::read(pkt)) return false;

  if (MII_MSG_HEARTBEAT_MSG_1 == pkt.msg_id) {
    /*memcpy(&(X_[pkt.node_id][JntType::KNEE]), pkt.data + 0, sizeof(short));
    memcpy(&(X_[pkt.node_id][JntType::HIP]),  pkt.data + 2, sizeof(short));
    memcpy(&(X_[pkt.node_id][JntType::YAW]),  pkt.data + 4, sizeof(short));*/
    int offset = 0;
    for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
        memcpy(&(X_[pkt.node_id][type]), pkt.data + offset, sizeof(short));
        offset += sizeof(short);
    }
  }

  return true;
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::MotorPcan, middleware::Label)
