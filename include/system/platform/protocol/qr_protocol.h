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

#define INVALID_ID (0xFF)

/*////////////////////////////////////////////////////////
The message id define:
        15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
to Node  0| 0       NODE ID          | COMMAND ID
to Group 0| 1       GROUP ID         | COMMAND ID
to Host  1| NODE ID               |    FEEDBACK ID
////////////////////////////////////////////////////////*/

#define MII_MSG_ID_BITS_SIZE      (16)
#define MII_MSG_ID_BYTE_SIZE      (2)
#define MII_HOST_MSG_MASK         (0x8000)
#define MII_HOST_MSG_NODE_ID_MASK (0x7F80)
#define MII_MSG_NODE_ID_MASK      (0x3FC0)
#define MII_MSG_GROUP_MASK        (0x4000)
#define MII_MSG_GROUP_ID_MASK     (0x3FC0)
#define MII_TO_NODE_TEMPLATE      (0x0000)
#define MII_TO_GROUP_TEMPLATE     (0x4000)
#define MII_TO_HOST_TEMPLATE      (0x8000)

///! define the command id
#define MII_CMD_JOINT_POS         (0x0001)
#define MII_CMD_JOINT_VEL         (0x0002)
#define MII_CMD_JOINT_TOR         (0x0003)

///! define the feedback id
#define MII_FB_JOINT_POS          (0x0001)
#define MII_FB_JOINT_VEL          (0x0002)
#define MII_FB_JOINT_TOR          (0x0003)
#define MII_FB_POWER_STATE        (0x0004)


} /* namespace quadruped_robot_driver */

#endif /* INCLUDE_MIDDLEWARE_PROTOCOL_QR_PROTOCOL_H_ */
