/*
 * usb.cpp
 *
 *  Created on: Oct 11, 2017
 *      Author: bibei
 */

#include <system/platform/propagate/usb.h>

#include "system/foundation/cfg_reader.h"

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>

namespace middleware {

MiiString g_imu_file    = "/dev/ttyUSB0";
int       g_imu_baud    = 9600;
int       g_counter     = 0;
const int MAX_TRY_TIMES = 1;
const int MAX_BUF_SIZE  = 1024;

USBChannel::USBChannel(const MiiString& l)
: Propagate(l), opened_(false), imu_fd_(-1),
  read_status_(-1), read_buf_(new char[MAX_BUF_SIZE]),
  buf_top_(read_buf_), buf_btm_(read_buf_),
  BUF_EOF_(read_buf_ + MAX_BUF_SIZE), imu_node_id_(0x00) {
  ; // Nothing to do here.
}

bool USBChannel::init() {
  if (!Propagate::init()) return false;

  auto cfg = MiiCfgReader::instance();
  cfg->get_value(getLabel(), "channel", g_imu_file);
  cfg->get_value(getLabel(), "baud",    g_imu_baud);
  cfg->get_value(getLabel(), "imu_node",imu_node_id_);
  return true;
}

USBChannel::~USBChannel() {
  stop();
  if (nullptr != read_buf_) {
    delete read_buf_;
    read_buf_ = nullptr;
  }
}

bool USBChannel::start() {
  opened_ = false;
  // try to 10 times
  for (g_counter = 0; g_counter < MAX_TRY_TIMES; ++g_counter) {
    imu_fd_ = open(g_imu_file.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    opened_ = (-1 != imu_fd_);
    if (!opened_) {
      LOG_WARNING << "(" << g_counter + 1 << "/" << MAX_TRY_TIMES
          << ") Initialize USB FAIL, error code: " << errno << "("
          << strerror(errno) << "), Waiting 500ms... ...";
      // Waiting 500ms
      usleep(500000);
    } else {
      LOG_INFO << "Initialize USB OK!";
      return opened_;
    }
  }

  LOG_ERROR << "Initialize USB FAIL!!!";
  return opened_;
}

void USBChannel::stop() {
  opened_ = false;
  // try to 10 times
  for (g_counter = 0; g_counter < MAX_TRY_TIMES; ++g_counter) {
    int err = close(imu_fd_);
    if (-1 != err){
      LOG_WARNING << "(" << g_counter + 1 << "/" << MAX_TRY_TIMES
          << ") Stopping USB FAIL, error code: " << errno << "("
          << strerror(errno) << "), Waiting 500ms... ...";
      // Waiting 500ms
      usleep(500000);
    } else {
      LOG_INFO << "Stopping USB OK!";
      return;
    }
  }

  LOG_ERROR << "Stopping USB FAIL!!!";
}

///! The helper method calculates the sum byte for up message.
inline bool __check_sum(const char* data) {
  char sum = data[0];
  // every message contains 11 bytes.
  const char* DATA_EOF = data + USB_UP_MESSAGE_SIZE - 1;
  while (++data != DATA_EOF) sum += *data;
  return (*data == sum);
}

bool USBChannel::read(Packet& pkt) {
  if (!opened_) {
    // LOG_FIRST_N(WARNING, 10000) << "The USB has not been launched, or initialized fail.";
    return false;
  }
  pkt.node_id = imu_node_id_;

  while (true) { // loop until parse an message or read error!
    char* offset = buf_btm_;
    while ((buf_top_ - offset) > USB_UP_MESSAGE_SIZE) {
      if ( (MII_USB_UP_HEADER != *offset++)
          || (*offset < MII_USB_UP_ID_TIME)
          || (*offset > MII_USB_UP_ID_ACCURACY)
          || (!__check_sum(offset - 1)) ) {
        // LOG_WARNING << "The USB message has error!";
        continue;
      }

      pkt.msg_id = *offset++;
      pkt.size   = USB_UP_MESSAGE_DATA_SIZE;
      memcpy(pkt.data, offset, USB_UP_MESSAGE_DATA_SIZE);
      offset += (USB_UP_MESSAGE_DATA_SIZE + 1); // one byte sum and then is the new data, so plus one
      buf_btm_ = offset;
      // printf("0x%02X : 0x%02X : 0x%02X : 0x%02X\n", read_buf_, buf_btm_, buf_top_, BUF_EOF_);
      return true;
    } // end while (offset < buf_top_)

    
    // printf("0x%02X 0x%02X 0x%02X 0x%02X\n", read_buf_, buf_btm_, buf_top_, BUF_EOF_);
    if ((BUF_EOF_ - buf_top_) < 2*USB_UP_MESSAGE_SIZE) {
      int __size = buf_top_ - buf_btm_;
      memcpy(read_buf_, buf_btm_, __size);
      buf_btm_ = read_buf_;
      buf_top_ = read_buf_ + __size;
    }

    read_status_ = ::read(imu_fd_, buf_top_, BUF_EOF_ - buf_top_);
    if (read_status_ <= 0) return false;
    buf_top_ += read_status_;
    // printf("READ: %d, 0x%02X vs 0x%02X\n", read_status_, buf_btm_, buf_top_);
  }
}

bool USBChannel::write(const Packet&) { return false; }

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::USBChannel, middleware::Label)
CLASS_LOADER_REGISTER_CLASS(middleware::USBChannel, middleware::Propagate)
