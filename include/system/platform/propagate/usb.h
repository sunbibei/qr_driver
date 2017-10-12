/*
 * usb.h
 *
 *  Created on: Oct 11, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_USB_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_USB_H_

#include <system/platform/propagate/propagate.h>

namespace middleware {

class USBChannel: public Propagate {
public:
  USBChannel(const MiiString& l = "usb");
  virtual bool init() override;

  virtual ~USBChannel();

public:
  virtual bool start() override;
  virtual void stop()  override;

  virtual bool read (class Packet&) override;

protected:
  bool  opened_;
  int   imu_fd_;
  int   read_status_;

  char*       read_buf_;
  // pointer helper
  char*       buf_top_;
  char*       buf_btm_;
  const char* BUF_EOF_;

  unsigned char imu_node_id_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_PROPAGATE_USB_H_ */
