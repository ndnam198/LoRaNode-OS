#ifndef __DATA_FORMAT_H
#define __DATA_FORMAT_H

#include "lora.h"
#include "main.h"

/* Node address must be in rage [0:0xFE] */
#define NODE1_ADDRESS   0x12u   /* Address node 1 */
#define NODE2_ADDRESS   0x13u   /* Address node 1 */
#define NODE3_ADDRESS   0x14u   /* Address node 2 */
#define NODE4_ADDRESS   0x15u   /* Address node 2 */
#define GATEWAY_ADDRESS 0xFFu   /* Adress Gateway */


enum MSG_INDEX
{
    INDEX_SOURCE_ID = 0u,     /* Header field, Tranceiver ID */
    INDEX_DEST_ID = 1u,           /* Header field, Receiver ID */
    INDEX_MSG_TYPE = 2u,          /* Header field, Type of msg, refer @MSG_TYPE enumeration */
    INDEX_MSG_STATUS = 3u,        /* Header field, Status of msg, refer @MSG_STATUS enumeration */
    INDEX_SEQUENCE_ID = 4u,       /* Header field, Message's sequence ID */
    INDEX_DATA_LOCATION = 5u,     /* Data field, refer to @DATA_LOCATION enumeration */
    INDEX_DATA_RELAY_STATE = 6u, /* Data field, either On or Off */
    INDEX_DATA_ERR_CODE = 7u,     /* Data field, refer to @DATA_ERR_CODE */
    INDEX_DATA_TIME_ALIVE = 8u,   /* Data field, time elapsed since light is controlled to On state */
    INDEX_UNDEFINED = 9u,             /*  */
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
    RELAY_STATE_ON = 0u,
    RELAY_STATE_OFF = 1u,
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
    LOCATION_UNKNOWN,
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


typedef struct NodeData {
    uint8_t nodeID;
    NodeLocationTypeDef_t location;
    NodeStsTypedef_t relayState;
    NodeErrCodeTypeDef_t errCode;
} NodeTypedef_t;

#endif /* !__DATA_FORMAT_H */
