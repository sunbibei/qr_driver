/*
 * propagate_manager.h
 *
 *  Created on: Aug 29, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_ROBOT_PROPAGATE_MANAGER_H_
#define INCLUDE_SYSTEM_ROBOT_PROPAGATE_MANAGER_H_

#include <system/platform/protocol/qr_protocol.h>
#include <system/platform/resource_manager.h>
#include "system/platform/propagate/propagate.h"

#include <thread>
#include <mutex>

namespace middleware {

class PropagateManager: public ResourceManager<Propagate> {
public:
  virtual ~PropagateManager();
  static PropagateManager* instance();

public:
  /**
   * @brief This method will starts a propagate thread for reading and writing Packets
   * @return Return true if everything is right, or return false.
   */
  bool run();
  /**
   * @brief This method is the main function for propagate thread.
   */
  void updatePktsQueues();

  bool readPackets (std::vector<Packet>&);
  bool writePackets(const std::vector<Packet>&);

protected:
  PropagateManager();
  static PropagateManager* instance_;

  // The interval time between twice RW.(in ms)
  std::chrono::milliseconds  propa_interval_;
  bool                       thread_alive_;
  // size_t                     pkts_queue_size;
  std::mutex          lock_4_send_;
  std::mutex          lock_4_recv_;
  std::vector<Packet> pkts_queue_4_send_;
  std::vector<Packet> pkts_queue_4_recv_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_PROPAGATE_MANAGER_H_ */
