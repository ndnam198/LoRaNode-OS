#ifndef __DATA_FORMAT_H
#define __DATA_FORMAT_H

#include "lora.h"
#include "main.h"
#include "misc.h"

/* Node address must be in rage [0:0xFE] */
#define NODE1_ADDRESS   0x12u   /* Address node 1 */
#define NODE2_ADDRESS   0x13u   /* Address node 1 */
#define NODE3_ADDRESS   0x14u   /* Address node 2 */
#define NODE4_ADDRESS   0x15u   /* Address node 2 */
#define GATEWAY_ADDRESS 0xFFu   /* Adress Gateway */


enum MSG_INDEX
{
    INDEX_SOURCE_ID = 0u,          /* Header field, Tranceiver ID */
    INDEX_DEST_ID = 1u,            /* Header field, Receiver ID */
    INDEX_MSG_TYPE = 2u,           /* Header field, Type of msg, refer @MSG_TYPE enumeration */
    INDEX_MSG_STATUS = 3u,         /* Header field, Status of msg, refer @MSG_STATUS enumeration */
    INDEX_SEQUENCE_ID = 4u,        /* Header field, Message's sequence ID */
    INDEX_DATA_LOCATION = 5u,      /* Data field, refer to @DATA_LOCATION enumeration */
    INDEX_DATA_RELAY_STATE = 6u,   /* Data field, either On or Off */
    INDEX_DATA_ERR_CODE = 7u,      /* Data field, refer to @DATA_ERR_CODE */
    INDEX_COMMAND_OPCODE = 8u,     /* Specify command by OPCODE, refer @OPCODE*/
    INDEX_RESET_CAUSE = 9u,        /* Data field, refer to @reset_cause in misc.h */
    INDEX_PACKET_RSSI = 9u,
    INDEX_MAX = 10u,
};

typedef enum HEADER_MSG_TYPE
{
    MSG_TYPE_REQUEST = 0u,
    MSG_TYPE_RESPONSE = 1u,
    MSG_TYPE_NOTIF = 2u,
    MSG_TYPE_MAX,
} MsgTypeDef_t;

typedef enum HEADER_MSG_STATUS
{
    MSG_STS_NONE = 0u,
    MSG_STS_OK = 1u,
    MSG_STS_FAILED = 2u,
    MSG_STS_MAX,
} MsgStsTypeDef_t;

typedef enum HEADER_ACK
{
    NO_ACK = 0u,
    NACK = 1u,
    ACK = 2u,
} ACKTypeDef_t;

typedef enum DATA_RELAY
{
    RELAY_STATE_OFF = 0u,
    RELAY_STATE_ON = 1u,
    RELAY_STATE_NONE = 2u,
    RELAY_STATE_MAX,
} NodeStsTypedef_t;

typedef enum DATA_ERR_CODE
{
    ERR_CODE_NONE = 0u,
    ERR_CODE_LIGHT_ON_FAILED = 1u,
    ERR_CODE_LIGHT_OFF_FAILED = 2u,
    ERR_CODE_MAX,
} NodeErrCodeTypeDef_t;

typedef enum DATA_LOCATION
{
    LOCATION_NONE,
    LOCATION_GIAI_PHONG_1,
    LOCATION_GIAI_PHONG_2,
    LOCATION_GIAI_PHONG_3,
    LOCATION_GIAI_PHONG_4,
    LOCATION_GIAI_PHONG_5,
    LOCATION_GIAI_PHONG_6,
    LOCATION_GIAI_PHONG_7,
    LOCATION_GIAI_PHONG_8,
    LOCATION_GIAI_PHONG_9,
    LOCATION_GIAI_PHONG_10,
    LOCATION_MAX,
} NodeLocationTypeDef_t;

enum OPCODE {
    OPCODE_NONE = 0,
    OPCODE_REQUEST_STATE = 1,
    OPCODE_RESPOSNE_STATE = OPCODE_REQUEST_STATE + 100,
    OPCODE_REQUEST_RELAY_CONTROL = 2,
    OPCODE_RESPOSNE_RELAY_CONTROL = OPCODE_REQUEST_RELAY_CONTROL + 100,
    OPCODE_REQUEST_MCU_RESET = 3,
    OPCODE_RESPOSNE_MCU_RESET = OPCODE_REQUEST_MCU_RESET + 100,
    OPCODE_REQUEST_LOCATION_UPDATE = 4,
    OPCODE_RESPOSNE_LOCATION_UPDATE = OPCODE_REQUEST_LOCATION_UPDATE + 100,
};

typedef struct NodeData {
    uint8_t nodeID;
    NodeLocationTypeDef_t location;
    NodeStsTypedef_t relayState;
    NodeErrCodeTypeDef_t errCode;
} NodeTypedef_t;

#define PACK_RESPONSE_MSG(msg, node, msgSts, seqID, opcode)\
do {                                                    \
msg[INDEX_SOURCE_ID]        = node.nodeID;              \
msg[INDEX_DEST_ID]          = GATEWAY_ADDRESS;          \
msg[INDEX_MSG_TYPE]         = MSG_TYPE_RESPONSE;        \
msg[INDEX_MSG_STATUS]       = msgSts;                   \
msg[INDEX_SEQUENCE_ID]      = seqID;                    \
msg[INDEX_DATA_LOCATION]    = node.location;            \
msg[INDEX_DATA_RELAY_STATE] = node.relayState;          \
msg[INDEX_DATA_ERR_CODE]    = node.errCode;             \
msg[INDEX_COMMAND_OPCODE]   = opcode;                   \
msg[INDEX_PACKET_RSSI]      = 0;         \
} while (0)

#define PACK_NOTIF_MSG(msg, node, resetCause)\
do {                                              \
msg[INDEX_SOURCE_ID]        = node.nodeID;        \
msg[INDEX_DEST_ID]          = GATEWAY_ADDRESS;    \
msg[INDEX_MSG_TYPE]         = MSG_TYPE_NOTIF;     \
msg[INDEX_MSG_STATUS]       = MSG_STS_NONE;       \
msg[INDEX_SEQUENCE_ID]      = -1;                 \
msg[INDEX_DATA_LOCATION]    = node.location;      \
msg[INDEX_DATA_RELAY_STATE] = node.relayState;    \
msg[INDEX_DATA_ERR_CODE]    = node.errCode;       \
msg[INDEX_COMMAND_OPCODE]   = 0;                  \
msg[INDEX_RESET_CAUSE]      = resetCause ;        \
} while (0)

#endif /* !__DATA_FORMAT_H */
