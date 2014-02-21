#ifndef BLECTRL_H
#define BLECTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
// Default ANCS UUID
#define APPLEANCS_SERVICE_UUID         0xD0,0x00,0x2D,0x12,0x1E,0x4B,0x0F,0xA4,0x99,0x4E,0xCE,0xB5,0x31,0xF4,0x05,0x79
#define APPLEANCS_NOTIFICATION_UUID    0xBD,0x1D,0xA2,0x99,0xE6,0x25,0x58,0x8C,0xD9,0x42,0x01,0x63,0x0D,0x12,0xBF,0x9F
#define APPLEDATASRC_NOTIFICATION_UUID 0xFB,0x7B,0x7C,0xCE,0x6A,0xB3,0x44,0xBE,0xB5,0x4B,0xD6,0x24,0xE9,0xC6,0xEA,0x22
#define APPLECTRLPT_UUID               0xD9,0xD9,0xAA,0xFD,0xBD,0x9B,0x21,0x98,0xA8,0x49,0xE1,0x45,0xF3,0xD8,0xD1,0x69

// Profile Parameters
#define BLECTRL_STATUS                    1
#define BLECTRL_UNCLS_NOTIF               2
#define BLECTRL_COMMAND                   3
#define BLECTRL_SERV_UUID_LEN             4
#define BLECTRL_SERV_UUID                 5
#define BLECTRL_SERV_START_HDL            6
#define BLECTRL_SERV_END_HDL              7
#define BLECTRL_CHAR_UUID_LEN             8
#define BLECTRL_CHAR_UUID                 9
#define BLECTRL_CHAR_HDL                  10
#define BLECTRL_DATA_LEN                  11
#define BLECTRL_DATA                      12
  
// Profile UUIDs
#define BLECTRL_STATUS_UUID              0xFF11
#define BLECTRL_UNCLS_NOTIF_UUID         0xFF12
#define BLECTRL_COMMAND_UUID             0xFF13
#define BLECTRL_SERV_UUID_LEN_UUID       0xFF14
#define BLECTRL_SERV_UUID_UUID           0xFF15
#define BLECTRL_SERV_START_HDL_UUID      0xFF16
#define BLECTRL_SERV_END_HDL_UUID        0xFF17
#define BLECTRL_CHAR_UUID_LEN_UUID       0xFF18
#define BLECTRL_CHAR_UUID_UUID           0xFF19
#define BLECTRL_CHAR_HDL_UUID            0xFF1A
#define BLECTRL_DATA_LEN_UUID            0xFF1B
#define BLECTRL_DATA_UUID                0xFF1C
  
// BLECTRL Service UUID
#define BLECTRL_SERVICE_UUID             0xFF10

// BLECTRL Profile Services bit fields
#define BLECTRL_SERVICE                  0x00000001
  
// BLECTRL Status
#define BLECTRL_STATUS_IDLE                             0
#define BLECTRL_DISCOVER_SERVICE_RSP                    1     // response of ATT_FIND_BY_TYPE_VALUE_RSP and found service in discover service *return numInfo
#define BLECTRL_DISCOVER_SERVICE_RSP_COMPLETE           2     // response of ATT_FIND_BY_TYPE_VALUE_RSP and complete in discover service
#define BLECTRL_DISCOVER_SERVICE_ERROR_RSP              3     // response of ATT_FIND_BY_TYPE_VALUE_RST AND ERROR_RSP in discover service *return errorCode
#define BLECTRL_DISCOVER_CHARACTERISTICS_RSP            4     // response of ATT_FIND_BY_TYPE_VALUE_RSP and found service in discover Chars *return numPairs
#define BLECTRL_READ_CHARACTERISTICS_ERROR_RSP          5     // response of ATT_ERROR_RSP in read char *return errorCode
#define BLECTRL_READ_CHARACTERISTICS_RSP                6     // response of read char
#define BLECTRL_WRITE_CHARACTERISTICS_ERROR_RSP         7     // response of ATT_ERROR_RSP in write char *return errorCode
#define BLECTRL_WRITE_CHARACTERISTICS_RSP               8     // response of write char

// BLECTRL Command Code
#define BLECTRL_CMD_IDLE                         0
#define BLECTRL_CMD_DISCOVER_SERVICE             1  
#define BLECTRL_CMD_DISCOVER_CHARACTERISTICS     2
#define BLECTRL_CMD_READ_CHARS_VALUE             3
#define BLECTRL_CMD_WRITE_CHARS_VALUE            4
#define BLECTRL_CMD_SUBSCRIBE_CHARACTERISTICS    5
#define BLECTRL_CMD_UNSUBSCRIBE_CHARACTERISTICS  6
#define BLECTRL_CMD_SUBS_UNSU_ANCS               128
#define BLECTRL_CMD_SUBS_UNSU_DATASRC            129
#define BLECTRL_CMD_WRITE_CONTROL_POINT          130

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */
// Callback when the device has been started.  Callback event to 
// the ask for a battery check.
typedef NULL_OK void (*blectrlChange_t)( uint8 paramID );

typedef struct
{
  blectrlChange_t        pfnBLECTRLChange;  // Called when Enabler attribute changes
} blectrlCBs_t;

/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * BLECTRL_AddService- Initializes the BLECTRL service by registering 
 *          GATT attributes with the GATT server. Only call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t BLECTRL_AddService( uint32 services );

/*
 * BLECTRL_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t BLECTRL_RegisterAppCBs( blectrlCBs_t *appCallbacks );


/*
 * BLECTRL_SetParameter - Set an BLECTRL Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t BLECTRL_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * BLECTRL_GetParameter - Get an BLECTRL Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t BLECTRL_GetParameter( uint8 param, void *value );

void BLECtrlStatusReport( uint8 status, uint8 moreInfo);
void BLECtrlDiscoverService(uint16 connHandle, uint8 task_id);
void BLECtrlDiscoverChars(uint16 connHandle, uint8 task_id);
void BLECtrlReadByHdl(uint16 connHandle, uint8 task_id);
void BLECtrlWriteByHdl(uint16 connHandle, uint8 task_id);
void BLECtrlSubscribeByHdl(uint16 connHandle, uint8 task_id, uint16 charHdl, uint8 subscribe);
void BLECtrlDiscoverCharsByUUID16(uint16 connHandle, uint8 task_id, uint8* uuid);
/*********************************************************************

*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BLECTRL_H */
