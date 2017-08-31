/*
 * propagate_manager.h
 *
 *  Created on: Aug 29, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_MANAGER_H_
#define INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_MANAGER_H_

#include <system/utils/resource_manager.h>
#include "propagate.h"

namespace middleware {

class PropagateManager: public ResourceManager<Propagate> {
public:
  virtual ~PropagateManager();
  static PropagateManager* instance();

protected:
  static PropagateManager* instance_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_MANAGER_H_ */
