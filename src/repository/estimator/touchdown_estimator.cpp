/*
 * planar_motion_estimator.cpp
 *
 *  Created on: Sep 12, 2017
 *      Author: bibei
 */

#include <repository/estimator/touchdown_estimator.h>
#include <system/foundation/cfg_reader.h>

namespace middleware {

TouchdownEstimator::TouchdownEstimator(const MiiString& _l)
  : Label(_l) {
  ; // Nothing to do here
}

bool TouchdownEstimator::init() {
  return true;
}

TouchdownEstimator::~TouchdownEstimator() { }

void TouchdownEstimator::input(double _new_data) { }

bool TouchdownEstimator::eval() {
  LOG_WARNING << "You don't should call this method with super's empty implement";
  return false;
}

} /* namespace middleware */
