/*
 * touchdown.h
 *
 *  Created on: Aug 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_TOUCHDOWN_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_TOUCHDOWN_H_

#include "system/label/label.h"

namespace middleware {

class TouchDown : public Label {
  friend class LegNode;
public:
  TouchDown(class TiXmlElement*);
  ~TouchDown();

  /**
   * Interface for user layer
   */
  double touchdown_data();

protected:

  /**
   * Interface for communication layer
   */
  // data = count * scale + offset
  void updateTouchdownState(short count);

  unsigned char  msg_id_;
  class TDState* td_state_;

  double         scale_;
  double         offset_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_TOUCHDOWN_H_ */
