/*
 * pcan_fake.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: bibei
 */

#include <system/platform/propagate/pcan_fake.h>
#include "system/foundation/cfg_reader.h"
#include <stdio.h>
#include <chrono>
#include <ctime>
#include <iomanip>

namespace middleware {

FILE*   g_r_fd   = nullptr;
FILE*   g_w_fd   = nullptr;
time_t* g_time_t = nullptr;
tm*     g_tm_    = nullptr;
bool    g_repeat = false;

#define INVALID_BYTE  (0x88)
#define ONLY_ONE_PKT  (0x11)
#define PACKET_W_FORMAT ("%4d-%02d-%02d, %02d:%02d:%02d\tID: 0x%02X, LEN: %d, DATA: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n")
#define PACKET_R_FORMAT ("ID: 0x%x, LEN: %d, DATA: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n")

PcanChannelFake::PcanChannelFake(const MiiString&) { }
bool PcanChannelFake::init() {
  auto cfg = MiiCfgReader::instance();

  MiiString filename;
  if (cfg->get_value(getLabel(), "output", filename))
    g_w_fd = fopen(filename.c_str(), "w");
  if (cfg->get_value(getLabel(), "input", filename)) {
    g_r_fd = fopen(filename.c_str(), "r");
    cfg->get_value(getLabel(), "repeat", g_repeat);
  }

  g_time_t = new time_t;
  g_tm_    = new tm;

  return PcanChannel::init();
}

PcanChannelFake::~PcanChannelFake() {
  if (nullptr != g_time_t) {
    delete g_time_t;
    g_time_t = nullptr;
  }
  if (nullptr != g_tm_) {
    delete g_tm_;
    g_tm_ = nullptr;
  }
  if (nullptr != g_r_fd) {
    fclose(g_r_fd);
    g_r_fd = nullptr;
  }
  if (nullptr != g_w_fd) {
    fclose(g_w_fd);
    g_w_fd = nullptr;
  }
}

bool PcanChannelFake::start() {
  LOG_INFO << "PcanChannelFake start. ";
  LOG_INFO << "Input  FD: " << g_r_fd;
  LOG_INFO << "Output FD: " << g_w_fd;
  return true;
}

bool PcanChannelFake::write(const Packet& pkt) {
  // static int g_w_err_count = 0;
  // if (!connected_) { return connected_; }
  // This code aims to compatible with the old protocol
  // TODO It should be updated.
  msg_4_send_->ID  = pkt.node_id;
  msg_4_send_->LEN = pkt.size + 3;
  memset(msg_4_send_->DATA, '\0', sizeof(msg_4_send_->DATA));
  msg_4_send_->DATA[0] = pkt.msg_id;
  msg_4_send_->DATA[1] = ONLY_ONE_PKT;
  msg_4_send_->DATA[2] = INVALID_BYTE;
  memcpy(msg_4_send_->DATA + 3, pkt.data, pkt.size * sizeof(BYTE));

  printf(PACKET_W_FORMAT,
       g_tm_->tm_year + 1900, g_tm_->tm_mon, g_tm_->tm_mday, g_tm_->tm_hour,
       g_tm_->tm_min, g_tm_->tm_sec, msg_4_send_->ID, (int)msg_4_send_->LEN,
       msg_4_send_->DATA[0], msg_4_send_->DATA[1], msg_4_send_->DATA[2],
       msg_4_send_->DATA[3], msg_4_send_->DATA[4], msg_4_send_->DATA[5],
       msg_4_send_->DATA[6], msg_4_send_->DATA[7]);

  time(g_time_t);
  g_tm_ = std::localtime(g_time_t);
  if (g_w_fd)
    fprintf(g_w_fd, PACKET_W_FORMAT, g_tm_->tm_year + 1900, g_tm_->tm_mon,
        g_tm_->tm_mday, g_tm_->tm_hour, g_tm_->tm_min, g_tm_->tm_sec,
        msg_4_send_->ID, (int)msg_4_send_->LEN, msg_4_send_->DATA[0],
        msg_4_send_->DATA[1], msg_4_send_->DATA[2], msg_4_send_->DATA[3],
        msg_4_send_->DATA[4], msg_4_send_->DATA[5], msg_4_send_->DATA[6],
        msg_4_send_->DATA[7]);

  return true;
}

bool PcanChannelFake::read(Packet& pkt) {
  if (g_r_fd) {
    if (feof(g_r_fd)) {
      if (g_repeat) {
        fseek(g_r_fd, 0, SEEK_SET);
      } else {
        fclose(g_r_fd);
        g_r_fd = nullptr;
      }
    }
    fscanf(g_r_fd, PACKET_R_FORMAT,
        &msg_4_recv_->ID, (int*)&msg_4_recv_->LEN,
         (int*)(msg_4_recv_->DATA + 0), (int*)(msg_4_recv_->DATA + 1),
         (int*)(msg_4_recv_->DATA + 2), (int*)(msg_4_recv_->DATA + 3),
         (int*)(msg_4_recv_->DATA + 4), (int*)(msg_4_recv_->DATA + 5),
         (int*)(msg_4_recv_->DATA + 6), (int*)(msg_4_recv_->DATA + 7));
  } else {
    LOG_EVERY_N(ERROR, 10) << "Read file FAIL!!!";
    return false;
  }

  if (false)
    printf("ID: 0x%02X, LEN: %d, DATA: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      msg_4_recv_->ID, (int)msg_4_recv_->LEN,
      msg_4_recv_->DATA[0], msg_4_recv_->DATA[1], msg_4_recv_->DATA[2], msg_4_recv_->DATA[3],
      msg_4_recv_->DATA[4], msg_4_recv_->DATA[5], msg_4_recv_->DATA[6], msg_4_recv_->DATA[7]);
  // This code aims to compatible with the old protocol
  // TODO It should be updated.
  if (msg_4_recv_->LEN < 3) {
    LOG_EVERY_N(ERROR, 10) << "Error Message from can bus, this length of message data is "
        << msg_4_recv_->LEN;
    return false;
  }
  pkt.node_id = msg_4_recv_->ID;
  pkt.msg_id  = msg_4_recv_->DATA[0];
  pkt.size    = msg_4_recv_->LEN - 3;
  memset(pkt.data, '\0', sizeof(pkt.data));
  memcpy(pkt.data, msg_4_recv_->DATA + 3, pkt.size * sizeof(BYTE));
  return true;
}

} /* namespace middleware */


#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::PcanChannelFake, middleware::Label)
CLASS_LOADER_REGISTER_CLASS(middleware::PcanChannelFake, middleware::Propagate)