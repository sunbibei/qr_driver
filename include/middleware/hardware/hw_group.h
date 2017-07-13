/*
 * hw_group.h
 *
 *  Created on: Jul 13, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_HW_GROUP_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_HW_GROUP_H_

#include <middleware/util/composite.h>
#include <middleware/hardware/hw_unit.h>

namespace middleware {

class HwGroup: public Composite<HwUnit> {
public:
  HwGroup();
  virtual ~HwGroup();

  virtual void init();
  virtual void publish();
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_HW_GROUP_H_ */
