/*
 * pcan_fake.h
 *
 *  Created on: Sep 7, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_PCAN_FAKE_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_PCAN_FAKE_H_

#include <system/platform/propagate/pcan.h>

namespace middleware {

class PcanChannelFake: public PcanChannel {
public:
  PcanChannelFake(const MiiString& l = "pcan");
  ~PcanChannelFake();
  virtual bool init() override;

  virtual bool start() override;
  virtual bool write(const class Packet&) override;
  virtual bool read (class Packet&)       override;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_PROPAGATE_PCAN_FAKE_H_ */
