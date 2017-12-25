/*
 * pcan_fake.h
 *
 *  Created on: Sep 7, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_PCAN_FAKE_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_PCAN_FAKE_H_

#include "system/platform/propagate/arm_pcan.h"
#include <atomic>

namespace middleware {

class PcanFake: public ArmPcan {
public:
  PcanFake(const MiiString& l = "pcan");
  ~PcanFake();
  virtual bool init() override;

  virtual bool start() override;
  virtual bool write(const class Packet&) override;
  virtual bool read (class Packet&)       override;

protected:
  std::atomic_bool is_swap_;
  TPCANMsg         mid_buffer_;

  time_t* r_time_t_;
  time_t* w_time_t_;
  tm*     r_tm_;
  tm*     w_tm_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_PROPAGATE_PCAN_FAKE_H_ */
