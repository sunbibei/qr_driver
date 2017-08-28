/*
 * hw_manager.h
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_HARDWARE_HW_MANAGER_H_
#define INCLUDE_MIDDLEWARE_HARDWARE_HW_MANAGER_H_

#include "middleware/util/resource_manager.h"

namespace middleware {

class HwManager : public ResourceManager<HwUnit> {
public:
  // After all of the hw_unit instance, call it.
  virtual bool init();

  // tick method
  virtual void tick();
protected:
  // class PropagateManager*                s_propagate_manager_;
  // store all of the hw_unit which order by id
  std::vector<class HwUnit*>             hw_list_by_id_;
  // store all of the hw_unit which require to send some command
  std::vector<class HwUnit*>             hw_list_by_cmd_;
  // store all of the hw_unit which order by name
  std::map<std::string, class HwUnit*>   hw_list_by_name_;

private:
  std::vector<Packet> packets_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_HARDWARE_HW_MANAGER_H_ */
