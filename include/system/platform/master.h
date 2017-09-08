/*
 * hw_manager.h
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_ROBOT_HW_MANAGER_H_
#define INCLUDE_SYSTEM_ROBOT_HW_MANAGER_H_

#include <system/platform/internal/resource_manager.h>
#include <system/platform/sw_node/sw_node.h>
#include <chrono>

namespace middleware {

class Master : public internal::ResourceManager<SWNode> {
  SINGLETON_DECLARE(Master)
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

  class PropagateManager*    propagate_manager_;
  class SWNodeManager*       sw_node_manager_;

private:
  MiiVector<Packet> packets_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_HW_MANAGER_H_ */
