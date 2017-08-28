/*
 * leg_node.h
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_LEG_NODE_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_LEG_NODE_H_

#include "hw_unit.h"

namespace middleware {

class LegNode: public HwUnit {
public:
  LegNode();
  virtual ~LegNode();

  virtual bool init(TiXmlElement*)      override;
  virtual void handleMsg(const Packet&) override;
  virtual bool generateCmd(Packet&)     override;

protected:
  // there are three joint in each leg
  LegType                   leg_;
  std::vector<class Joint*> joints_;
  class TouchDown*          td_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_LEG_NODE_H_ */
