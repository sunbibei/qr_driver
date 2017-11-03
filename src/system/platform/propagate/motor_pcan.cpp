/*
 * motor_pcan.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: bibei
 */

#include <system/platform/propagate/motor_pcan.h>
#include "system/foundation/cfg_reader.h"

namespace middleware {

const MiiString FAKE_PID_THREAD = "fake-pid";


MotorPcan::MotorPcan(const MiiString& l)
  : ArmPcan(l), new_target_(false), pid_alive_(false) {
  pids_.reserve(MAX_NODE_NUM);
  for (auto& pid : pids_)
    pid.reserve(JntType::N_JNTS);
}

bool MotorPcan::init() {
  if (!ArmPcan::init()) return false;

  int count = 0;
  unsigned char node_id = INVALID_BYTE;
  MiiString tag = Label::make_label(getLabel(), "pid_" + std::to_string(count));
  while(MiiCfgReader::instance()->get_value(tag, "node_id", node_id)) {
    auto_inst_pid(tag);
    tag = Label::make_label(getLabel(), "pid_" + std::to_string(++count));
  }

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

  return true;
  // return ArmPcan::write(pkt);
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
  curr_update_t_ = std::chrono::high_resolution_clock::now();
  dt_ = std::chrono::duration_cast<std::chrono::seconds>(
      curr_update_t_ - last_update_t_);
  updatePID(pkt.node_id);

  last_update_t_ = curr_update_t_;
  return true;
}

void MotorPcan::updatePID(unsigned char node_id) {
  Packet pkt = {bus_id_, node_id, MII_MSG_MOTOR_CMD_2, 6, {0}};

  int offset = 0;
  for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
    U_[node_id][type] = pids_[node_id][type].computeCommand(
        T_[node_id][type] - X_[node_id][type], dt_.count());

    memcpy(pkt.data + offset, U_[node_id] + type, sizeof(short));
    offset += sizeof(short);
  }
  ArmPcan::write(pkt);
}

void MotorPcan::auto_inst_pid(const MiiString& __p) {
  unsigned char node_id = INVALID_BYTE;
  auto cfg = MiiCfgReader::instance();
  cfg->get_value_fatal(__p, "node_id", node_id);
  bool is_insert = false;
  for (const auto& id : node_ids_) {
    if (id == node_id) is_insert = true;
  }
  if (!is_insert) node_ids_.push_back(node_id);
  JntType jnt = JntType::UNKNOWN_JNT;
  cfg->get_value_fatal(__p, "jnt", jnt);

  pids_[node_id][jnt] = Pid(__p);
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::MotorPcan, middleware::Label)
