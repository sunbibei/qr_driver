/*
 * propagate_imp_pcan.h
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#ifndef INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_
#define INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_

#include "propagate.h"

#define linux
#include <unistd.h>
#include <asm/types.h>

#include <PCANBasic.h>

namespace middleware {

class PcanChannel: public Propagate {
public:
  PcanChannel(const MiiString& l = "pcan");
  ~PcanChannel();
  virtual bool init() override;

  virtual bool start() override;
  virtual void stop()  override;

  virtual bool write(const class Packet&) override;
  virtual bool read (class Packet&)       override;

protected:
  TPCANMsg     send_msg_;
  TPCANMsg     recv_msg_;
  bool         connected_;
};

} /* namespace qr_driver */

#endif /* INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_ */
