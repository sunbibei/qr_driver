/*
 * leg_node.h
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#ifndef INCLUDE_APPS_HW_UNIT_LEG_NODE_H_
#define INCLUDE_APPS_HW_UNIT_LEG_NODE_H_

#include "system/platform/hw_unit/hw_unit.h"
#include "system/foundation/utf.h"

namespace middleware {

class LegNode: public HwUnit {
public:
  LegNode(const MiiString& __l = Label::null);
  virtual bool init() override;

  virtual ~LegNode();

  virtual void handleMsg(const Packet&)          override;
  virtual bool generateCmd(std::vector<Packet>&) override;

protected:
  // there are three joint in each leg
  LegType                   leg_;
  std::vector<class Joint*> joints_;
  std::vector<class Joint*> joints_by_id_;
  class TouchDown*          td_;
};

} /* namespace middleware */

#endif /* INCLUDE_APPS_HW_UNIT_LEG_NODE_H_ */