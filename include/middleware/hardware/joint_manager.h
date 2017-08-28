/*
 * joint_manager.h
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_JOINT_MANAGER_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_JOINT_MANAGER_H_

#include "middleware/util/resource_manager.h"
#include "joint.h"

namespace middleware {

class JointManager : public ResourceManager<Joint> {
public:

};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_JOINT_MANAGER_H_ */
