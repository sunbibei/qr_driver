/*
 * touchdown.h
 *
 *  Created on: Aug 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_TOUCHDOWN_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_TOUCHDOWN_H_

#include "middleware/util/proto/dragon.pb.h"
#include "system/label/label.h"

namespace middleware {

class TouchDown : public Label {
  friend class LegNode;
public:
  TouchDown(MiiStringConstRef __l = Label::null);

  // 妥协方案
  virtual bool init() override;
  ~TouchDown();

  /**
   * Interface for user layer
   */
  double touchdown_data();
  const LegType& leg_type() const;

protected:

  /**
   * Interface for communication layer
   */
  // data = count * scale + offset
  void updateTouchdownState(short count);

  LegType        leg_type_;

  unsigned char  msg_id_;
  class TDState* td_state_;

  double         scale_;
  double         offset_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_TOUCHDOWN_H_ */
