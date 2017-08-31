/*
 * propagate.h
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_
#define INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_

#include "system/label/label.h"
#include <vector>

namespace middleware {

class Propagate : public Label {
  friend class PropagateManager;
public:
  Propagate(const MiiString& name);
  virtual ~Propagate();

  // 本通信方式的名称， 该名称作为Hw_Unit的Channel参数
  std::string propa_name_;

  /**
   * 等待子类实现的函数s
   */
  virtual bool init();
  virtual bool start();
  virtual void stop();

  virtual bool write(class Packet*);
  virtual bool read(class Packet*);

};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_ */
