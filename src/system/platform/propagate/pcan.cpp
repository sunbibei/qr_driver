/*
 * propagate_imp_pcan.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#include <system/platform/protocol/qr_protocol.h>
#include "system/platform/propagate/pcan.h"

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
  : Propagate(l), connected_(false) {
  send_msg_.MSGTYPE = PCAN_MESSAGE_STANDARD;
}

PcanChannel::~PcanChannel() {
  stop();
  ;//  Nothing need to dealloc
}

bool PcanChannel::init() {
  return Propagate::init();
}

bool PcanChannel::start() {
  connected_ = false;
  // try to 10 times
  for (g_times_count = 0; g_times_count < MAX_TRY_TIMES; ++g_times_count) {
    // g_status_ = CAN_Initialize(g_channel, g_baud_rate, g_type, g_port, g_interrupt);
    TPCANStatus status = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_500K, 0, 0, 0);
    connected_ = (PCAN_ERROR_OK == status);
    if (!connected_) {
      LOG_WARNING << "(" << g_times_count + 1 << "/10) Initialize CAN FAIL, "
          "status code: " << status << ", Waiting 500ms... ...";
      // Waiting 500ms
      usleep(500000);
    } else {
      LOG_INFO << "Initialize CAN OK!";
      return connected_;
    }
  }

  LOG_ERROR << "Initialize CAN FAIL!!!";
  return connected_;
}

void PcanChannel::stop() {
  connected_ = false;
  // try to 10 times
  for (g_times_count = 0; g_times_count < MAX_TRY_TIMES; ++g_times_count) {
    g_status_ = CAN_Uninitialize(g_channel);
    if (PCAN_ERROR_OK != g_status_){
      LOG_WARNING << "(" << g_times_count + 1 << "/10) Uninitialize CAN FAIL, "
          "status code: " << g_status_ << ", Waiting 500ms... ...";
      // Waiting 500ms
      usleep(500000);
    } else {
      LOG_INFO << "Uninitialize CAN OK!";
      return;
    }
  }

  LOG_ERROR << "Uninitialize CAN FAIL!!!";
}

bool PcanChannel::write(const Packet& pkt) {
  send_msg_.MSGTYPE = PCAN_MESSAGE_STANDARD;
  send_msg_.ID      = MII_MSG_FILL_TO_NODE_MSG(pkt.node_id, pkt.msg_id);
  send_msg_.LEN     = pkt.size;
  memset(send_msg_.DATA, '\0', 8 * sizeof(BYTE));
  memcpy(send_msg_.DATA, pkt.data, send_msg_.LEN * sizeof(BYTE));
  // static int g_w_err_count = 0;
  // if (!connected_) { return connected_; }
  // This code aims to compatible with the old protocol
  // TODO It should be updated.
  /*msg_4_send_.ID  = pkt.node_id;
  msg_4_send_.LEN = pkt.size + 3;
  msg_4_send_.DATA[0] = pkt.msg_id;
  msg_4_send_.DATA[1] = ONLY_ONE_PKT;
  msg_4_send_.DATA[2] = INVALID_BYTE;
  memcpy(msg_4_send_.DATA + 3, pkt.data, pkt.size * sizeof(BYTE));*/

  // try to 10 times
  for (g_times_count = 0; g_times_count < MAX_TRY_TIMES; ++g_times_count) {
    g_status_ = CAN_Write(g_channel, &send_msg_);
    if (PCAN_ERROR_OK != g_status_){
      LOG_WARNING << "(" << g_times_count + 1 << "/10) Write CAN message FAIL, "
          "status code: " << g_status_ << ", Waiting 50ms... ...";
      // Waiting 50ms
      usleep(50000);
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
  // memset(&recv_msg_, '\0', sizeof(TPCANMsg));
  auto tmp = recv_msg_;
  while (PCAN_ERROR_OK != (g_status_ = CAN_Read(PCAN_USBBUS1, &tmp, NULL))) {
    if (++g_times_count < MAX_TRY_TIMES) {
      LOG_WARNING << "read again!(" << g_times_count << "/"
          << MAX_TRY_TIMES << "), error code: " << g_status_;
      usleep(5000);
    } else {
      LOG_ERROR << "The pcan channel has read fail!";
      return false;
    }
  }
  if (0x06 == tmp.ID) {
    LOG_DEBUG << "FUCKING!";
    stop();
    usleep(5000);
    start();
  }

  printf("  - COUNT: %05d ID:0x%03X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
        read_counter++, (int)tmp.ID, (int)tmp.LEN,
        (int)tmp.DATA[0], (int)tmp.DATA[1],
        (int)tmp.DATA[2], (int)tmp.DATA[3],
        (int)tmp.DATA[4], (int)tmp.DATA[5],
        (int)tmp.DATA[6], (int)tmp.DATA[7]);
  /*if (true)
    printf("  - COUNT: %05d ID:0x%03X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      read_counter++, (int)recv_msg_.ID, (int)recv_msg_.LEN,
      (int)recv_msg_.DATA[0], (int)recv_msg_.DATA[1],
      (int)recv_msg_.DATA[2], (int)recv_msg_.DATA[3],
      (int)recv_msg_.DATA[4], (int)recv_msg_.DATA[5],
      (int)recv_msg_.DATA[6], (int)recv_msg_.DATA[7]);*/

  return false;

  pkt.node_id = MII_MSG_EXTRACT_NODE_ID(recv_msg_.ID);
  pkt.msg_id  = MII_MSG_EXTRACT_MSG_ID(recv_msg_.ID);
  pkt.size    = recv_msg_.LEN;
  memset(pkt.data, '\0', 8 * sizeof(char));
  memcpy(pkt.data, recv_msg_.DATA, pkt.size * sizeof(char));

  // This code aims to compatible with the old protocol
  // TODO It should be updated.
  /*if (msg_4_recv_.LEN < 3) {
    LOG_EVERY_N(ERROR, 10) << "Error Message from can bus, this length of message data is "
        << msg_4_recv_.LEN;
    return false;
  }
  pkt.node_id = msg_4_recv_.ID;
  pkt.msg_id  = msg_4_recv_.DATA[0];
  pkt.size    = msg_4_recv_.LEN - 3;
  memcpy(pkt.data, msg_4_recv_.DATA + 3, pkt.size * sizeof(BYTE));*/
  return true;
}

} /* namespace qr_driver */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::PcanChannel, middleware::Label)
CLASS_LOADER_REGISTER_CLASS(middleware::PcanChannel, middleware::Propagate)

