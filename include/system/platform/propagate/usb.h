/*
 * usb.h
 *
 *  Created on: Oct 11, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_USB_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_USB_H_

#include "system/platform/propagate/propagate.h"

namespace middleware {

class UsbPropagate: public Propagate {
public:
  UsbPropagate(const MiiString& l = "usb");
  virtual bool init() override;

  virtual ~UsbPropagate();

public:
  virtual bool start() override;
  virtual void stop()  override;

  /*virtual bool write(const Packet&) override;
  virtual bool read (class Packet&) override;*/

protected:
  bool  opened_;
  int   usb_fd_;
  unsigned char node_id_;

  struct UsbConfig {
    MiiString file_name;
    int       baud_rate;
  } usb_config_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_PROPAGATE_USB_H_ */
