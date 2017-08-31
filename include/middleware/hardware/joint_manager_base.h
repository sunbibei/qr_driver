/*
 * joint_manager.h
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_JOINT_MANAGER_BASE_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_JOINT_MANAGER_BASE_H_

#include <system/utils/resource_manager.h>
#include "joint.h"

namespace middleware {

class JointManagerBase : public ResourceManager<Joint> { };

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_JOINT_MANAGER_BASE_H_ */
