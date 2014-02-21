#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "archergeneral.h"
#include "OSAL_Clock.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        14

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */


// General Service UUID
CONST uint8 generalServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(GENERAL_SERVICE_UUID), HI_UINT16(GENERAL_SERVICE_UUID)
};

// App Connect UUID
CONST uint8 appConnectUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(APP_CONNECT_UUID), HI_UINT16(APP_CONNECT_UUID)
};

// RSSI VALUE UUID
CONST uint8 rssiValueUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(RSSI_VALUE_UUID), HI_UINT16(RSSI_VALUE_UUID)
};

// RSSI VALUE UUID
CONST uint8 ancsStateUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ANCS_STATE_UUID), HI_UINT16(ANCS_STATE_UUID)
};

// General Clock UUID
CONST uint8 generalClockUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(GENERAL_CLOCK_UUID), HI_UINT16(GENERAL_CLOCK_UUID)
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
static generalCBs_t *general_AppCBs = NULL;


/*********************************************************************
 * Profile Attributes - variables
 */

// General Service attribute
static CONST gattAttrType_t generalService = { ATT_BT_UUID_SIZE, generalServUUID };

// APP Connect Characteristic Properties
static uint8 appConnectCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// APP Connect Characteristic Value
static uint8 appConnect = NOT_CONNECTED;

// APP Connect Characteristic user description
static uint8 appConnectUserDesc[12] = "APP Connect\0";

// RSSI Value Characteristic Properties
static uint8 rssiValueCharProps = GATT_PROP_READ;

// RSSI Value Characteristic Value
static int8 rssiValue = 0;

// RSSI Value Characteristic user description
static uint8 rssiValueUserDesc[11] = "RSSI Value\0";

// ANCS State Characteristic Properties
static uint8 ancsStateCharProps = GATT_PROP_NOTIFY;

// ANCS State Characteristics
static uint8 ancsState;

// ANCS State Characteristic Configs
static gattCharCfg_t ancsStateConfig[GATT_MAX_NUM_CONN];

// ANCS State Characteristic user descriptions
static uint8 ancsStateCharUserDesc[12] = "ANCS State\0";

// General Clock Characteristic Properties
static uint8 generalClockCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// General Clock Characteristic Value
static uint32 generalClock = 0;

// General Clock Characteristic user description
static uint8 generalClockUserDesc[14] = "General Clock\0";

/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t generalAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // General Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                   /* permissions */
    0,                                  /* handle */
    (uint8 *)&generalService                /* pValue */
  },
     
    // APP Connect Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &appConnectCharProps 
    },

      // APP Connect Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, appConnectUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        (uint8 *)&appConnect 
      },

      // APP Connect User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)appConnectUserDesc 
      },
      
    // RSSI Value Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &rssiValueCharProps 
    },

      // RSSI Value Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, rssiValueUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8 *)(&rssiValue)
      },

      // RSSI Value User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&rssiValueUserDesc 
      },
      
    // ANCS State Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ancsStateCharProps 
    },
    
      // ANCS State Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, ancsStateUUID },
        0, 
        0, 
        (uint8 *)&ancsState
      },
      
      // ANCS State Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)ancsStateConfig 
      },

      // ANCS State Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        ancsStateCharUserDesc
      },  
      
    // General Clock Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &generalClockCharProps 
    },
    
      // General Clock Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, generalClockUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        (uint8 *)&generalClock
      },
      
      // General Clock User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)generalClockUserDesc 
      },
      
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 general_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                               uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t general_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset );

static void general_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
//  General Service Callbacks
CONST gattServiceCBs_t  generalCBs =
{
  general_ReadAttrCB,  // Read callback function pointer
  general_WriteAttrCB, // Write callback function pointer
  NULL               // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      General_AddService
 *
 * @brief   Initializes the General service by
 *          registering GATT attributes with the GATT server. Only
 *          call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t General_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, ancsStateConfig );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( general_HandleConnStatusCB );  

  if ( services & GENERAL_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( generalAttrTbl, 
                                          GATT_NUM_ATTRS( generalAttrTbl ),
                                          &generalCBs );
  }

  return ( status );
}

/*********************************************************************
 * @fn      General_RegisterAppCBs
 *
 * @brief   Does the profile initialization.  Only call this function
 *          once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t General_RegisterAppCBs( generalCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    general_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


/*********************************************************************
 * @fn      General_SetParameter
 *
 * @brief   Set an General Profile parameter.
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
bStatus_t General_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case APP_CONNECT:
      if ( len == sizeof ( uint8 ) )
      {
        appConnect = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case RSSI_VALUE:
      if ( len == sizeof ( int8 ) ) 
      {
        rssiValue = *((int8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break; 
        
    case ANCS_STATE:
      if ( len == sizeof(uint8) ) 
      {      
        ancsState = *((uint8*)value);
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( ancsStateConfig, (uint8 *)&ancsState,
                                    FALSE, generalAttrTbl, GATT_NUM_ATTRS( generalAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      General_GetParameter
 *
 * @brief   Get an General Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t General_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case APP_CONNECT:
      *((uint8*)value) = appConnect;
      break;
      
    case ANCS_STATE:
      *((uint8*)value) = ancsState;
      break;
    
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          general_ReadAttr
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
static uint8 general_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
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
      case APP_CONNECT_UUID:
      case RSSI_VALUE_UUID:
      case ANCS_STATE_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
      case GENERAL_CLOCK_UUID:
        getUTCTime();
        *pLen = 4;
        pValue[0] = BREAK_UINT32( *((uint32 *)pAttr->pValue) , 0);
        pValue[1] = BREAK_UINT32( *((uint32 *)pAttr->pValue) , 1);
        pValue[2] = BREAK_UINT32( *((uint32 *)pAttr->pValue) , 2);
        pValue[3] = BREAK_UINT32( *((uint32 *)pAttr->pValue) , 3);
        break;
      
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
 * @fn      general_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle – connection message was received on
 * @param   pReq - pointer to request
 *
 * @return  Success or Failure
 */
static bStatus_t general_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notifyAPP = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case APP_CONNECT_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > 1 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
          else if ( pValue[0] != APP_CONNECTED )
            status = ATT_ERR_INVALID_VALUE;
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          *pCurValue = pValue[0];
          notifyAPP = APP_CONNECT;
        }
        break;
        
      case GENERAL_CLOCK_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != 4 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint32 *pCurValue = (uint32 *)pAttr->pValue;
          *pCurValue = BUILD_UINT32(pValue[0],pValue[1],pValue[2],pValue[3]);
          setUTCTime();
        }
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
  if ( (notifyAPP != 0xFF) && general_AppCBs && general_AppCBs->pfnGeneralChange )
    general_AppCBs->pfnGeneralChange(notifyAPP);  
  
  return ( status );
}

/*********************************************************************
 * @fn          general_HandleConnStatusCB
 *
 * @brief       General Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void general_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      //GATTServApp_InitCharCfg( connHandle, ancsMsgConfig );
    }
  }
}

/*********************************************************************
*********************************************************************/
void getUTCTime()
{
  generalClock = osal_getClock();
}

void setUTCTime()
{
  osal_setClock(generalClock);
}