/*
 * qr_protocol.h
 *
 *  Created on: Jul 18, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_PROTOCOL_QR_PROTOCOL_H_
#define INCLUDE_MIDDLEWARE_PROTOCOL_QR_PROTOCOL_H_

namespace middleware {

// if the type of communication is can
// uncomment the follow line
#define PACKET_CAN
#ifndef PACKET_CAN
#define DATA_SIZE (0)
#else
#define DATA_SIZE (8)
#endif

struct Packet {
  unsigned char node_id;
  unsigned char msg_id;
  unsigned int  size;
  char data[DATA_SIZE];
};

#define INVALID_BYTE (0x88)

/*////////////////////////////////////////////////////////
The message id define:
        10 09 08 07 06 05 04 03 02 01 00
to Node  0| 0|    NODE ID|        MSG ID|
to Group 0| 1|   GROUP ID|        MSG ID|
to Host  1| X|    NODE ID|        MSG ID|
////////////////////////////////////////////////////////*/

#define MII_MSG_ID_BITS_SIZE      (11)
#define MII_MSG_ID_BYTE_SIZE      (2)
///! The template of to node , to group and to host message(No setting node_id and msg_id)
#define MII_MSG_TEMPLATE_TO_NODE      (0x0000u)
#define MII_MSG_TEMPLATE_TO_GROUP     (0x0200u)
#define MII_MSG_TEMPLATE_TO_HOST      (0x0400u)

///! The utils of process message
// Judge the type of message
#define MII_MSG_IS_TO_HOST(msg)       (((msg) & (0x0400u)) == (0x0400u))
#define MII_MSG_IS_TO_NODE(msg)       (((msg) & (0x0600u)) == (0x0000u))
#define MII_MSG_IS_TO_GROUP(msg)      (((msg) & (0x0600u)) == (0x0200u))
// Extracting the node_id, group_id and msg_id from the message
#define MII_MSG_EXTRACT_NODE_ID(msg)  (((msg) & (0x01E0u)) >> 5)
#define MII_MSG_EXTRACT_GROUP_ID(msg) MII_MSG_EXTRACT_NODE_ID(msg)
#define MII_MSG_EXTRACT_MSG_ID(msg)    ((msg) & (0x001Fu))
// Filling the node_id and msg_id into the given msg which contains the
// message head such as where will to go.
#define __MII_MSG_FILL_TO_MSG(msg, node_id, msg_id) \
    ((((msg) & 0x0600u) | (((node_id) & 0x0Fu) << 5)) | ((msg_id) & (0x1Fu)))
// Filling the node id and msg_id into the to host message.
#define MII_MSG_FILL_TO_HOST_MSG(node_id, msg_id) \
    __MII_MSG_FILL_TO_MSG(0x0400u, node_id, msg_id)
// Filling the node_id and msg_id into the to node message.
#define MII_MSG_FILL_TO_NODE_MSG(node_id, msg_id) \
    __MII_MSG_FILL_TO_MSG(0x0000u, node_id, msg_id)
// Filling the node_id and msg_id into the to group message.
#define MII_MSG_FILL_TO_GROUP_MSG(node_id, msg_id) \
    __MII_MSG_FILL_TO_MSG(0x0200u, node_id, msg_id)
// Filling the node_id into the given message
#define MII_MSG_FILL_NODE_ID(msg, node_id) \
    __MII_MSG_FILL_TO_MSG(msg, node_id, MII_MSG_EXTRACT_MSG_ID(msg))
// Filling the msg_id into the given message
#define MII_MSG_FILL_MSG_ID(msg, msg_id) \
    __MII_MSG_FILL_TO_MSG(msg, MII_MSG_EXTRACT_NODE_ID(msg), msg_id)

///! define the msg id
#define MII_MSG_HEARTBEAT_MSG     (0x01u)
///! Return joint position and touchdown data from leg node
#define MII_MSG_HEARTBEAT_MSG_1   (0x02u)

///!
#define MII_MSG_COMMON_DATA       (0x10u)
///! The joint command
#define MII_MSG_COMMON_DATA_1     (0x11u)

} /* namespace quadruped_robot_driver */


/**
 * TEST PROGRAM and RESULTS
 */

// #define TEST_PROTOCOL
#ifdef TEST_PROTOCOL

 #include <stdio.h>
 using namespace middleware;

 int main(int argc, char* argv[]) {
  int count = 0;
  unsigned int id = MII_MSG_FILL_TO_HOST_MSG(0x02, 0x01);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
      MII_MSG_EXTRACT_MSG_ID(id));
  // 0: id: 0x441, toHost: 0x01, node id: 0x02, msg id: 0x01

  id = MII_MSG_FILL_NODE_ID(id, 0x0A);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
      MII_MSG_EXTRACT_MSG_ID(id));
  // 1: id: 0x541, toHost: 0x01, node id: 0x0A, msg id: 0x01

  id = MII_MSG_FILL_MSG_ID(id, 0x1F);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
      MII_MSG_EXTRACT_MSG_ID(id));
  // 2: id: 0x55F, toHost: 0x01, node id: 0x0A, msg id: 0x1F

  id = MII_MSG_FILL_TO_NODE_MSG(0x0F, 0x11);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
      MII_MSG_EXTRACT_MSG_ID(id));
  // 3: id: 0x1F1, toHost: 0x00, node id: 0x0F, msg id: 0x11

  id = MII_MSG_FILL_TO_GROUP_MSG(0x05, 0xAF);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
      MII_MSG_EXTRACT_MSG_ID(id));
  // 4: id: 0x2AF, toHost: 0x00, node id: 0x05, msg id: 0x0F

  id = MII_MSG_FILL_NODE_ID(id, 0x0A);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
      MII_MSG_EXTRACT_MSG_ID(id));
  // 5: id: 0x34F, toHost: 0x00, node id: 0x0A, msg id: 0x0F

  id = MII_MSG_FILL_MSG_ID(id, 0xFF);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
      MII_MSG_EXTRACT_MSG_ID(id));
  // 6: id: 0x35F, toHost: 0x00, node id: 0x0A, msg id: 0x1F
  return 0;
 }
#endif


#endif /* INCLUDE_MIDDLEWARE_PROTOCOL_QR_PROTOCOL_H_ */
