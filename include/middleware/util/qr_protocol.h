/*
 * qr_protocol.h
 *
 *  Created on: Jul 18, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_UTIL_QR_PROTOCOL_H_
#define INCLUDE_MIDDLEWARE_UTIL_QR_PROTOCOL_H_

/*////////////////////////////////////////////////////////
The message id define:
        15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
to Node  0| 0       NODE ID          | COMMAND ID
to Group 0| 1       GROUP ID         | COMMAND ID
to Host  1| NODE ID               |    FEEDBACK ID
////////////////////////////////////////////////////////*/

#define MII_MSG_ID_BITS_SIZE (16)
#define MII_MSG_ID_BYTE_SIZE (2)
#define MII_HOST_MSG_MASK    (0x8000)




#endif /* INCLUDE_MIDDLEWARE_UTIL_QR_PROTOCOL_H_ */
