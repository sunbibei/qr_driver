/*
 * imu_usb.h
 *
 *  Created on: Nov 1, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_IMU_USB_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_IMU_USB_H_

#include "system/platform/propagate/usb.h"

namespace middleware {

class ImuUsb: public UsbPropagate {
public:
  ImuUsb(const MiiString& l = "usb_imu");
  virtual ~ImuUsb();

public:
  virtual bool write(const Packet&) override;
  virtual bool read (class Packet&) override;

protected:
  int         read_status_;

  char*       read_buf_;
  // pointer helper
  char*       buf_top_;
  char*       buf_btm_;
  const char* BUF_EOF_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_PROPAGATE_IMU_USB_H_ */
