/*
 * usb.cpp
 *
 *  Created on: Oct 11, 2017
 *      Author: bibei
 */

#include <system/platform/propagate/usb.h>

#include "foundation/cfg_reader.h"

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>

namespace middleware {

int       g_counter     = 0;
const int MAX_TRY_TIMES = 10;

UsbPropagate::UsbPropagate(const MiiString& l)
: Propagate(l), opened_(false),
  usb_fd_(-1), node_id_(0xFF) {
  ; // Nothing to do here.
}

bool UsbPropagate::init() {
  if (!Propagate::init()) return false;

  auto cfg = MiiCfgReader::instance();
  cfg->get_value(getLabel(), "channel", usb_config_.file_name);
  cfg->get_value(getLabel(), "baud",    usb_config_.baud_rate);
  cfg->get_value(getLabel(), "node_id", node_id_);
  return true;
}

UsbPropagate::~UsbPropagate() {
  stop();
}

bool UsbPropagate::start() {
  opened_ = false;
  // try to 10 times
  for (g_counter = 0; g_counter < MAX_TRY_TIMES; ++g_counter) {
    usb_fd_ = open(usb_config_.file_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    opened_ = (-1 != usb_fd_);
    if (!opened_) {
      LOG_DEBUG << "(" << g_counter + 1 << "/" << MAX_TRY_TIMES
          << ") Initialize USB FAIL, error code: " << errno << "("
          << strerror(errno) << "), Waiting 500ms... ...";
      // Waiting 500ms
      usleep(500000);
    } else {
      LOG_DEBUG << "Initialize USB OK!";
      return opened_;
    }
  }

  LOG_ERROR << "Initialize USB FAIL!!!";
  return opened_;
}

void UsbPropagate::stop() {
  if ((-1 == usb_fd_) || !opened_) return;

  // try to 10 times
  for (g_counter = 0; g_counter < MAX_TRY_TIMES; ++g_counter) {
    int err = close(usb_fd_);
    if (-1 != err){
      LOG_DEBUG << "(" << g_counter + 1 << "/" << MAX_TRY_TIMES
          << ") Stopping USB FAIL, error code: " << errno << "("
          << strerror(errno) << "), Waiting 500ms... ...";
      // Waiting 500ms
      usleep(500000);
    } else {
      usb_fd_ = -1;
      opened_ = false;
      LOG_INFO << "Stopping USB OK!";
      return;
    }
  }

  LOG_ERROR << "Stopping USB FAIL!!!";
}

} /* namespace middleware */
