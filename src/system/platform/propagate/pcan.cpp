/*
 * propagate_imp_pcan.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#include <system/platform/protocol/qr_protocol.h>
#include "system/platform/propagate/pcan.h"
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

PcanChannel::PcanChannel(const MiiString& l)
  : Propagate(l), msg_4_send_(new TPCANMsg), msg_4_recv_(new TPCANMsg),
    connected_(false) {
  msg_4_send_->MSGTYPE = PCAN_MESSAGE_STANDARD;
}

PcanChannel::~PcanChannel() {
  if (nullptr == msg_4_send_) {
    delete msg_4_send_;
    msg_4_send_ = nullptr;
  }
  if (nullptr == msg_4_recv_) {
    delete msg_4_recv_;
    msg_4_recv_ = nullptr;
  }
}

bool PcanChannel::init() {
  return Propagate::init();
}

bool PcanChannel::start() {
  connected_ = false;
  // try to 10 times
  for (g_times_count = 0; g_times_count < MAX_TRY_TIMES; ++g_times_count) {
    g_status_ = CAN_Initialize(g_channel, g_baud_rate, g_type, g_port, g_interrupt);
    connected_ = (PCAN_ERROR_OK == g_status_);
    if (!connected_) {
      LOG_WARNING << "(" << g_times_count + 1 << "/10) Initialize CAN FAIL, "
          "status code: " << g_status_ << ", Waiting 500ms... ...";
      // Waiting 500ms
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
      LOG_INFO << "Initialize CAN OK!";
      return connected_;
    }
  }

  LOG_ERROR << "Initialize CAN FAIL!!!";
  return connected_;
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
  connected_ = false;
}

bool PcanChannel::write(const Packet& pkt) {
  msg_4_send_->MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg_4_send_->ID      = MII_MSG_FILL_TO_NODE_MSG(pkt.node_id, pkt.msg_id);
  msg_4_send_->LEN     = pkt.size;
  memset(msg_4_send_->DATA, '\0', 8 * sizeof(BYTE));
  memcpy(msg_4_send_->DATA, pkt.data, msg_4_send_->LEN * sizeof(BYTE));
  // static int g_w_err_count = 0;
  // if (!connected_) { return connected_; }
  // This code aims to compatible with the old protocol
  // TODO It should be updated.
  /*msg_4_send_->ID  = pkt.node_id;
  msg_4_send_->LEN = pkt.size + 3;
  msg_4_send_->DATA[0] = pkt.msg_id;
  msg_4_send_->DATA[1] = ONLY_ONE_PKT;
  msg_4_send_->DATA[2] = INVALID_BYTE;
  memcpy(msg_4_send_->DATA + 3, pkt.data, pkt.size * sizeof(BYTE));*/

  // try to 10 times
  for (g_times_count = 0; g_times_count < MAX_TRY_TIMES; ++g_times_count) {
    g_status_ = CAN_Write(g_channel, msg_4_send_);
    if (PCAN_ERROR_OK != g_status_){
      LOG_WARNING << "(" << g_times_count + 1 << "/10) Write CAN message FAIL, "
          "status code: " << g_status_ << ", Waiting 50ms... ...";
      // Waiting 50ms
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } else
      // g_w_err_count = 0;
      return true;
  }

  LOG_EVERY_N(ERROR, 10) << "Write CAN FAIL!!!";
  return false;
}

unsigned int read_counter = 0;

bool PcanChannel::read(Packet& pkt) {
  g_times_count = 0;
  memset(msg_4_recv_, '\0', sizeof(TPCANMsg));
  while (PCAN_ERROR_QRCVEMPTY == (g_status_ = CAN_Read(PCAN_USBBUS1, msg_4_recv_, NULL))) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (g_times_count++ >= MAX_TRY_TIMES) {
      LOG_WARNING << "read again! (" << g_times_count << "/" << MAX_TRY_TIMES << ")";
      break;
    }
  }
  if (PCAN_ERROR_OK != g_status_) return false;

  if (true)
    printf("%d -- ID: 0x%02X, LEN: %d, DATA: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      read_counter++, msg_4_recv_->ID, (int)msg_4_recv_->LEN,
      msg_4_recv_->DATA[0], msg_4_recv_->DATA[1], msg_4_recv_->DATA[2], msg_4_recv_->DATA[3],
      msg_4_recv_->DATA[4], msg_4_recv_->DATA[5], msg_4_recv_->DATA[6], msg_4_recv_->DATA[7]);

  pkt.node_id = MII_MSG_EXTRACT_NODE_ID(msg_4_recv_->ID);
  pkt.msg_id  = MII_MSG_EXTRACT_MSG_ID(msg_4_recv_->ID);
  pkt.size    = msg_4_recv_->LEN;
  memset(pkt.data, '\0', 8 * sizeof(char));
  memcpy(pkt.data, msg_4_recv_->DATA, pkt.size * sizeof(char));

  // This code aims to compatible with the old protocol
  // TODO It should be updated.
  /*if (msg_4_recv_->LEN < 3) {
    LOG_EVERY_N(ERROR, 10) << "Error Message from can bus, this length of message data is "
        << msg_4_recv_->LEN;
    return false;
  }
  pkt.node_id = msg_4_recv_->ID;
  pkt.msg_id  = msg_4_recv_->DATA[0];
  pkt.size    = msg_4_recv_->LEN - 3;
  memcpy(pkt.data, msg_4_recv_->DATA + 3, pkt.size * sizeof(BYTE));*/
  return true;
}


} /* namespace qr_driver */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::PcanChannel, middleware::Label)
CLASS_LOADER_REGISTER_CLASS(middleware::PcanChannel, middleware::Propagate)

