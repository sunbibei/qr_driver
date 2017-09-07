/*
 * hw_manager.h
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_ROBOT_HW_MANAGER_H_
#define INCLUDE_SYSTEM_ROBOT_HW_MANAGER_H_

#include <system/platform/resource_manager.h>
#include "system/platform/hw_unit/hw_unit.h"
#include <chrono>

namespace middleware {

class HwManager : public ResourceManager<HwUnit> {
  SINGLETON_DECLARE(HwManager)
public:
  // After all of the hw_unit instance, call it.
  bool init();
  /**
   * @brief This method will starts a tick thread for deliver Packets
   * @return Return true if everything is right, or return false.
   */
  bool run();
protected:
  // tick method
  void tick();
  // The interval time between twice RW.(in ms)
  std::chrono::milliseconds  tick_interval_;
  bool                       thread_alive_;

  class PropagateManager*                propagate_manager_;
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

#endif /* INCLUDE_SYSTEM_ROBOT_HW_MANAGER_H_ */
