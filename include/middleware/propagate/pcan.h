/*
 * propagate_imp_pcan.h
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#ifndef INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_
#define INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_

#include "propagate.h"

namespace middleware {

class PcanChannel: public Propagate {
public:
  PcanChannel(const std::string& name = "pcan");
  virtual ~PcanChannel();

  virtual bool init(TiXmlElement*) override;
  virtual bool write(void*, size_t, uint32_t) override;
  virtual int  read(void*, size_t, uint32_t&) override;
  virtual void stop() override;

  virtual void check() override;
};

} /* namespace qr_driver */

#endif /* INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_ */
