#ifndef LEDCTRL_H
#define LEDCTRL_H

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
#define ARCHER_LED_CTRL       1  // uint8[6], byte0-LED selection, byte1-color, byte2-flash times, byte3-wait time, byte4-on time, byte5-off time
  
// Profile UUIDs
#define ARCHER_LED_CTRL_UUID  0xFFB1    // uint8[6], byte0-LED selection, byte1-color, byte2-flash times, byte3-wait time, byte4-on time, byte5-off time
  
// Accelerometer Service UUID
#define LED_SERVICE_UUID            0xFFB0
  
// Accelerometer Profile Services bit fields
#define LED_SERVICE                 0x00000001

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
typedef NULL_OK void (*ledUpdate_t)( void );

typedef struct
{
  ledUpdate_t        pfnLedUpdate;  // Called when attribute changes
} ledCBs_t;

/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * Accel_AddService- Initializes the Accelerometer service by registering 
 *          GATT attributes with the GATT server. Only call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t Led_AddService( uint32 services );

/*
 * Accel_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Led_RegisterAppCBs( ledCBs_t *appCallbacks );


/*
 * Accel_SetParameter - Set an Accelerometer Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t Led_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * Accel_GetParameter - Get an Accelerometer Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t Led_GetParameter( uint8 param, void *value );

void ProcessLEDCtrl(void);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* LEDCTRL_H */
