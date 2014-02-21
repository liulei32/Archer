#ifndef GENERAL_H
#define GENERAL_H

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

// Profile Parameters
#define APP_CONNECT                   1  // RW uint8 - Profile Attribute value
#define RSSI_VALUE                    2  // RO uint8[8] - Profile Attribute value
#define ANCS_STATE                    3  // Notification
#define GENERAL_CLOCK                 4  // Clock
  
// Profile UUIDs
#define APP_CONNECT_UUID              0xFFF1
#define RSSI_VALUE_UUID               0xFFF2
#define ANCS_STATE_UUID               0xFFF3
#define GENERAL_CLOCK_UUID            0xFFF4
  
// General Service UUID
#define GENERAL_SERVICE_UUID          0xFFF0

// General Profile Services bit fields
#define GENERAL_SERVICE                  0x00000001

// APP_CONNECT States define
#define NOT_CONNECTED             0x00
#define APP_CONNECTED             0x01
#define SYS_CONNECTED             0x02

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
typedef NULL_OK void (*generalChange_t)( uint8 paramID );

typedef struct
{
  generalChange_t        pfnGeneralChange;  // Called when Enabler attribute changes
} generalCBs_t;

/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * General_AddService- Initializes the General service by registering 
 *          GATT attributes with the GATT server. Only call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t General_AddService( uint32 services );

/*
 * General_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t General_RegisterAppCBs( generalCBs_t *appCallbacks );


/*
 * General_SetParameter - Set an General Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t General_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * General_GetParameter - Get an General Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t General_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/
void getUTCTime();
void setUTCTime();
#ifdef __cplusplus
}
#endif

#endif /* GENERAL_H */
