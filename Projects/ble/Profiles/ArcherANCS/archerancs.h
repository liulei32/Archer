#ifndef ANCS_H
#define ANCS_H

#ifdef __cplusplus
extern "C"
{
#endif
  
#define MAX_LEN                           20

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define ANCS_ENABLER                  1  // RW uint8 - Profile Attribute value
#define ANCS_MSG                      2  // uint8[8] - Profile Attribute value
#define DATASRC_ENABLER               3  // RW uint8 - Profile Attribute value
#define DATASRC_MSG                   4  // uint[MAX_LEN]
#define CONTROL_POINT                 5  // uint[MAX_LEN]
#define CONTROL_POINT_LEN             128  // Reserved, used in getparameter only
  
// Profile UUIDs
#define ANCS_ENABLER_UUID             0xFF01
#define ANCS_MSG_UUID                 0xFF02
#define DATASRC_ENABLER_UUID          0xFF03
#define DATASRC_MSG_UUID              0xFF04
#define CONTROL_POINT_UUID            0xFF05
  
// ANCS Service UUID
#define ANCS_SERVICE_UUID             0xFF00

// ANCS Profile Services bit fields
#define ANCS_SERVICE                  0x00000001

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
typedef NULL_OK void (*ancsChange_t)( uint8 paramID );

typedef struct
{
  ancsChange_t        pfnANCSChange;  // Called when Enabler attribute changes
} ancsCBs_t;

/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * ANCS_AddService- Initializes the ANCS service by registering 
 *          GATT attributes with the GATT server. Only call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t ANCS_AddService( uint32 services );

/*
 * ANCS_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t ANCS_RegisterAppCBs( ancsCBs_t *appCallbacks );


/*
 * ANCS_SetParameter - Set an ANCS Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t ANCS_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * ANCS_GetParameter - Get an ANCS Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t ANCS_GetParameter( uint8 param, void *value );

void ANCSWriteCtrlPoint(uint16 connHandle, uint8 task_id, uint16 charHdl);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ANCS_H */
