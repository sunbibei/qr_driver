/*
 * joint.h
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_

namespace middleware {

struct Joint {
  Joint(TiXmlElement*);

  LegType leg_;
  JntType jnt_;
  unsigned char       cmd_id_;
  class JointState*   joint_state_;
  class JointCommand* joint_command_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_JOINT_H_ */
