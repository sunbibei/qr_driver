/*
 * propagate_imp_pcan.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#include "middleware/propagate/pcan.h"
#include "middleware/util/qr_protocol.h"

#include <thread>

namespace middleware {

#define INVALID_BYTE  (0x88)
#define ONLY_ONE_PKT  (0x11)

TPCANStatus   g_status_      = PCAN_ERROR_OK;
TPCANHandle   g_channel      = PCAN_USBBUS1;
TPCANBaudrate g_baud_rate    = PCAN_BAUD_500K;
TPCANType     g_type         = 0;
DWORD         g_port         = 0;
WORD          g_interrupt    = 0;

const unsigned int MAX_TRY_TIMES = 10;
unsigned int       g_times_count = 0;

PcanChannel::PcanChannel(MiiStringConstRef l)
  : Propagate(l), msg_4_send_(new TPCANMsg), msg_4_recv_(new TPCANMsg) {
  msg_4_send_->MSGTYPE = PCAN_MESSAGE_STANDARD;
}

bool PcanChannel::init() {
  return Propagate::init();
}

bool PcanChannel::start() {
  // try to 10 times
  for (g_times_count = 0; g_times_count < MAX_TRY_TIMES; ++g_times_count) {
    g_status_ = CAN_Initialize(g_channel, g_baud_rate, g_type, g_port, g_interrupt);
    if (PCAN_ERROR_OK != g_status_){
      LOG_WARNING << "(" << g_times_count + 1 << "/10) Initialize CAN FAIL, "
          "status code: " << g_status_ << ", Waiting 500ms... ...";
      // Waiting 500ms
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
      LOG_INFO << "Initialize CAN OK!";
      return true;
    }
  }

  LOG_ERROR << "Initialize CAN FAIL!!!";
  return false;
}

void PcanChannel::stop() {
  // try to 10 times
  for (g_times_count = 0; g_times_count < MAX_TRY_TIMES; ++g_times_count) {
    g_status_ = CAN_Uninitialize(g_channel);
    if (PCAN_ERROR_OK != g_status_){
      LOG_WARNING << "(" << g_times_count + 1 << "/10) Uninitialize CAN FAIL, "
          "status code: " << g_status_ << ", Waiting 500ms... ...";
      // Waiting 500ms
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
      LOG_INFO << "Uninitialize CAN OK!";
      return;
    }
  }

  LOG_ERROR << "Uninitialize CAN FAIL!!!";
}

bool PcanChannel::write(const Packet& pkt) {
  // This code aims to compatible with the old protocol
  // TODO It should be updated.
  msg_4_send_->ID  = pkt.node_id;
  msg_4_send_->LEN = pkt.size + 3;
  msg_4_send_->DATA[0] = pkt.msg_id;
  msg_4_send_->DATA[1] = ONLY_ONE_PKT;
  msg_4_send_->DATA[2] = INVALID_BYTE;
  memcpy(msg_4_send_->DATA + 3, pkt.data, pkt.size * sizeof(BYTE));

  // try to 10 times
  for (g_times_count = 0; g_times_count < MAX_TRY_TIMES; ++g_times_count) {
    g_status_ = CAN_Write(g_channel, msg_4_send_);
    if (PCAN_ERROR_OK != g_status_){
      LOG_WARNING << "(" << g_times_count + 1 << "/10) Write CAN message FAIL, "
          "status code: " << g_status_ << ", Waiting 50ms... ...";
      // Waiting 50ms
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } else
      return true;
  }

  LOG_ERROR << "Write CAN FAIL!!!";
  return false;
}

bool PcanChannel::read(Packet& pkt) {
  g_times_count = 0;
  while ((g_status_ = CAN_Read(g_channel, msg_4_recv_, nullptr)) == PCAN_ERROR_QRCVEMPTY){
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (g_times_count++ >= MAX_TRY_TIMES) return false;
  }

  // This code aims to compatible with the old protocol
  // TODO It should be updated.
  if (msg_4_recv_->LEN < 3) {
    LOG_WARNING << "Error Message from can bus, this length of message data is "
        << msg_4_recv_->LEN;
    return false;
  }
  pkt.node_id = msg_4_recv_->ID;
  pkt.msg_id  = msg_4_recv_->DATA[0];
  pkt.size    = msg_4_recv_->LEN - 3;
  memcpy(pkt.data, msg_4_recv_->DATA + 3, pkt.size * sizeof(BYTE));
  return true;
}


} /* namespace qr_driver */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::PcanChannel, middleware::Label)
CLASS_LOADER_REGISTER_CLASS(middleware::PcanChannel, middleware::Propagate)

