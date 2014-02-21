/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "hal_archerled.h"
#include "archerled.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        4

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Led Service UUID
CONST uint8 ledServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(LED_SERVICE_UUID), HI_UINT16(LED_SERVICE_UUID)
};

// Archer LED UUID
CONST uint8 archerLedCtrlUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ARCHER_LED_CTRL_UUID), HI_UINT16(ARCHER_LED_CTRL_UUID)
};
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static ledCBs_t *led_AppCBs = NULL;


/*********************************************************************
 * Profile Attributes - variables
 */

// Led Service attribute
static CONST gattAttrType_t ledService = { ATT_BT_UUID_SIZE, ledServUUID };

// Archer LED Characteristic Properties
static uint8 archerLedCtrlCharProps = GATT_PROP_WRITE;

// Archer LED Characteristic Value
static uint8 archerLedCtrl[6];

// Archer LED Characteristic user description
static uint8 archerLedCtrlUserDesc[10] = "Led Ctrl\0";

// Client Characteristic configuration. Each client has its own instantiation
// of the Client Characteristic Configuration. Reads of the Client Characteristic
// Configuration only shows the configuration for that client and writes only
// affect the configuration of that client.

/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t ledAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Led Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                   /* permissions */
    0,                                  /* handle */
    (uint8 *)&ledService                /* pValue */
  },
  
    // Archer Led Ctrl Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &archerLedCtrlCharProps 
    },

      // Archer Led Ctrl Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, archerLedCtrlUUID },
        GATT_PERMIT_WRITE, 
        0,
        archerLedCtrl
      },

      // Archer Led Ctrl User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&archerLedCtrlUserDesc 
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 led_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                               uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t led_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset );

static void led_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
//  Accelerometer Service Callbacks
CONST gattServiceCBs_t  ledCBs =
{
  led_ReadAttrCB,  // Read callback function pointer
  led_WriteAttrCB, // Write callback function pointer
  NULL               // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Led_AddService
 *
 * @brief   Initializes the Led service by
 *          registering GATT attributes with the GATT server. Only
 *          call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Led_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( led_HandleConnStatusCB );  

  if ( services & LED_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( ledAttrTbl, 
                                          GATT_NUM_ATTRS( ledAttrTbl ),
                                          &ledCBs );
  }

  return ( status );
}

/*********************************************************************
 * @fn      Led_RegisterAppCBs
 *
 * @brief   Does the profile initialization.  Only call this function
 *          once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Led_RegisterAppCBs( ledCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    led_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


/*********************************************************************
 * @fn      Led_SetParameter
 *
 * @brief   Set an Accelerometer Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Led_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ( ret );
}

/*********************************************************************
 * @fn      Accel_GetParameter
 *
 * @brief   Get an Accelerometer Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Led_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          led_ReadAttr
 *
 * @brief       Read an attribute.
 *
 * @param       pAttr - pointer to attribute
 * @param       pLen - length of data to be read
 * @param       pValue - pointer to data to be read
 * @param       signature - whether to include Authentication Signature
 *
 * @return      Success or Failure
 */
static uint8 led_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                               uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  uint16 uuid;
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {    
    // 16-bit UUID
    uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those types for reads
      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }


  return ( status );
}

/*********************************************************************
 * @fn      led_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle – connection message was received on
 * @param   pReq - pointer to request
 *
 * @return  Success or Failure
 */
static bStatus_t led_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notify = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case ARCHER_LED_CTRL_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != 6 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          VOID osal_memcpy( pCurValue, pValue, len );
        }
        ProcessLEDCtrl();
        break;
      
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;      
      default:
          // Should never get here!
          status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }  

  // If an attribute changed then callback function to notify application of change
  if ( (notify != 0xFF) && led_AppCBs && led_AppCBs->pfnLedUpdate )
    led_AppCBs->pfnLedUpdate();
  
  return ( status );
}

/*********************************************************************
 * @fn          accel_HandleConnStatusCB
 *
 * @brief       Accelerometer Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void led_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      ;
    }
  }
}


/*********************************************************************
*********************************************************************/
void ProcessLEDCtrl()
{
  if (archerLedCtrl[0] != 0)
    ArcherLedSet(archerLedCtrl[0]-1,archerLedCtrl[1],archerLedCtrl[2],archerLedCtrl[3],archerLedCtrl[4],archerLedCtrl[5]);
  else
  {
    for (uint8 i=0; i<NUM_LED; i++)
      ArcherLedSet(i,archerLedCtrl[1],archerLedCtrl[2],archerLedCtrl[3],archerLedCtrl[4],archerLedCtrl[5]);
  }
    
}