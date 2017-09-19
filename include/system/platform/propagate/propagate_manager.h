/*
 * propagate_manager.h
 *
 *  Created on: Aug 29, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_ROBOT_PROPAGATE_MANAGER_H_
#define INCLUDE_SYSTEM_ROBOT_PROPAGATE_MANAGER_H_

#include "system/platform/protocol/qr_protocol.h"
#include "system/platform/internal/resource_manager.h"
#include "system/platform/propagate/propagate.h"
#include <mutex>

namespace middleware {

class PropagateManager: public internal::ResourceManager<Propagate> {
  SINGLETON_DECLARE(PropagateManager)
public:
  /**
   * @brief This method will starts a propagate thread for reading and writing Packets
   * @return Return true if everything is right, or return false.
   */
  bool run();
  /**
   * @brief This method is the main function for propagate thread.
   */
  void update();

  /**
   * @brief These methods offer the public function for user, User could obtains
   *        the incoming data through the follow @readPackets methods and sends
   *        the outgoing data through the @writePackets methods.
   */
  bool readPackets (MiiVector<Packet>&);
  bool writePackets(const MiiVector<Packet>&);

protected:
  // The interval time between twice RW.(in ms)
  std::chrono::milliseconds  propa_interval_;
  bool                       thread_alive_;
  // size_t                     pkts_queue_size;
  std::mutex          lock_4_send_;
  std::mutex          lock_4_recv_;
  MiiVector<Packet>   pkts_queue_4_send_;
  MiiVector<Packet>   pkts_queue_4_recv_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_PROPAGATE_MANAGER_H_ */
