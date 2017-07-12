/*
 * leg.h
 *
 *  Created on: Jul 11, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_LEG_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_LEG_H_

#include <middleware/hardware/hw_unit.h>

namespace middleware {

class Leg: public HwUnit {
public:
  Leg();
  virtual ~Leg();
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_LEG_H_ */
