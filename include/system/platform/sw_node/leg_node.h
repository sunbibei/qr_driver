/*
 * leg_node.h
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#ifndef INCLUDE_APPS_HW_UNIT_LEG_NODE_H_
#define INCLUDE_APPS_HW_UNIT_LEG_NODE_H_

#include <system/platform/sw_node/sw_node.h>
#include "system/foundation/utf.h"

namespace middleware {

class LegNode: public SWNode {
public:
  LegNode(const MiiString& __l = Label::null);
  virtual bool init() override;

  virtual ~LegNode();

  virtual void handleMsg(const Packet&)          override;
  virtual bool generateCmd(std::vector<Packet>&) override;

protected:
  void updateFromBuf(const unsigned char*);
  // there are three joint in each leg
  LegType                   leg_;
  std::vector<class Joint*> joints_by_type_;
  class ForceSensor*          td_;
  // The constant pointer of the joint command
  const double*             jnt_cmds_[JntType::N_JNTS];
  const JntCmdType*         jnt_mods_[JntType::N_JNTS];
};

} /* namespace middleware */

#endif /* INCLUDE_APPS_HW_UNIT_LEG_NODE_H_ */
