#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "archerancs.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        18

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */


// ANCS Service UUID
CONST uint8 ancsServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ANCS_SERVICE_UUID), HI_UINT16(ANCS_SERVICE_UUID)
};

// ANCS Enabler UUID
CONST uint8 ancsEnablerUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ANCS_ENABLER_UUID), HI_UINT16(ANCS_ENABLER_UUID)
};

// ANCS Msg UUID
CONST uint8 ancsMsgUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ANCS_MSG_UUID), HI_UINT16(ANCS_MSG_UUID)
};

// Data Source Enabler UUID
CONST uint8 dataSrcEnablerUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(DATASRC_ENABLER_UUID), HI_UINT16(DATASRC_ENABLER_UUID)
};

// Data Source Msg UUID
CONST uint8 dataSrcMsgUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(DATASRC_MSG_UUID), HI_UINT16(DATASRC_MSG_UUID)
};

// Control Point UUID
CONST uint8 controlPointUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(CONTROL_POINT_UUID), HI_UINT16(CONTROL_POINT_UUID)
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
static ancsCBs_t *ancs_AppCBs = NULL;


/*********************************************************************
 * Profile Attributes - variables
 */

// ANCS Service attribute
static CONST gattAttrType_t ancsService = { ATT_BT_UUID_SIZE, ancsServUUID };

// Enabler Characteristic Properties
static uint8 ancsEnabledCharProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_AUTHEN;

// Enabler Characteristic Value
static uint8 ancsEnabled = FALSE;

// Enabler Characteristic user description
static uint8 ancsEnabledUserDesc[13] = "ANCS Enable\0";

// ANCS Msg Characteristic Properties
static uint8 ancsMsgCharProps = GATT_PROP_NOTIFY;

// ANCS Msg Characteristics
static uint8 ancsMsg[MAX_LEN];
static uint8 ancsMsgLength = 0;

// Client Characteristic configuration. Each client has its own instantiation
// of the Client Characteristic Configuration. Reads of the Client Characteristic
// Configuration only shows the configuration for that client and writes only
// affect the configuration of that client.

// ANCS Msg Characteristic Configs
static gattCharCfg_t ancsMsgConfig[GATT_MAX_NUM_CONN];

// ANCS Characteristic user descriptions
static uint8 ancsMsgCharUserDesc[14] = "ANCS Message\0";

// Data Source Enabler Characteristic Properties
static uint8 dataSrcEnabledCharProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_AUTHEN;

// Data Source Enabler Characteristic Value
static uint8 dataSrcEnabled = FALSE;

// Data Source Enabler Characteristic user description
static uint8 dataSrcEnabledUserDesc[19] = "Data Source Enable\0";

// Data Source Msg Characteristic Properties
static uint8 dataSrcMsgCharProps = GATT_PROP_NOTIFY;

// Data Source Msg Characteristics
static uint8 dataSrcMsg[MAX_LEN];
static uint8 dataSrcMsgLength = 0;

// Client Characteristic configuration. Each client has its own instantiation
// of the Client Characteristic Configuration. Reads of the Client Characteristic
// Configuration only shows the configuration for that client and writes only
// affect the configuration of that client.

// Data Source Msg Characteristic Configs
static gattCharCfg_t dataSrcMsgConfig[GATT_MAX_NUM_CONN];

// Data Source Characteristic user descriptions
static uint8 dataSrcMsgCharUserDesc[20] = "Data Source Message\0";

// Control Point Characteristic Properties
static uint8 controlPointCharProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_AUTHEN;

// Control Point Characteristic Value
static uint8 controlPoint[MAX_LEN];
static uint8 controlPointLength;

// Control Point Characteristic user description
static uint8 controlPointUserDesc[19] = "ANCS Control Point\0";

/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t ancsAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // ANCS Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                   /* permissions */
    0,                                  /* handle */
    (uint8 *)&ancsService                /* pValue */
  },
      
    // ANCS Enabler Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ancsEnabledCharProps 
    },

      // ANCS Enable Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, ancsEnablerUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE | GATT_PERMIT_AUTHEN_WRITE, 
        0,
        &ancsEnabled 
      },

      // ANCS Enable User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&ancsEnabledUserDesc 
      },

    // ANCS Msg Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ancsMsgCharProps 
    },
      // ANCS Msg Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, ancsMsgUUID },
        0, 
        0, 
        (uint8 *)ancsMsg
      },
      
      // ANCS Msg Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)ancsMsgConfig 
      },

      // ANCS Msg Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        ancsMsgCharUserDesc
      },  
      
    // Data Source Enabler Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &dataSrcEnabledCharProps 
    },

      // Data Source Enable Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, dataSrcEnablerUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE | GATT_PERMIT_AUTHEN_WRITE, 
        0,
        &dataSrcEnabled 
      },

      // Data Source Enable User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&dataSrcEnabledUserDesc 
      },
      
    // Data Source Msg Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &dataSrcMsgCharProps 
    },
      // Data Source Msg Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, dataSrcMsgUUID },
        0, 
        0, 
        (uint8 *)dataSrcMsg
      },
      
      // Data Source Msg Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)dataSrcMsgConfig 
      },

      // Data Source Msg Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        dataSrcMsgCharUserDesc
      }, 
            
    // Control Point Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &controlPointCharProps 
    },

      // Control Point Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, controlPointUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE | GATT_PERMIT_AUTHEN_WRITE, 
        0,
        (uint8 *)controlPoint 
      },

      // Control Point User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)controlPointUserDesc 
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 ancs_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                               uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t ancs_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset );

static void ancs_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
//  ANCS Service Callbacks
CONST gattServiceCBs_t  ancsCBs =
{
  ancs_ReadAttrCB,  // Read callback function pointer
  ancs_WriteAttrCB, // Write callback function pointer
  NULL               // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ANCS_AddService
 *
 * @brief   Initializes the ANCS service by
 *          registering GATT attributes with the GATT server. Only
 *          call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t ANCS_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, ancsMsgConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, dataSrcMsgConfig );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( ancs_HandleConnStatusCB );  

  if ( services & ANCS_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( ancsAttrTbl, 
                                          GATT_NUM_ATTRS( ancsAttrTbl ),
                                          &ancsCBs );
  }

  return ( status );
}

/*********************************************************************
 * @fn      ANCS_RegisterAppCBs
 *
 * @brief   Does the profile initialization.  Only call this function
 *          once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t ANCS_RegisterAppCBs( ancsCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    ancs_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


/*********************************************************************
 * @fn      ANCS_SetParameter
 *
 * @brief   Set an ANCS Profile parameter.
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
bStatus_t ANCS_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  { 
    case ANCS_MSG:
      if ( len <= MAX_LEN ) 
      {      
        ancsMsgLength = len;
        VOID osal_memcpy( ancsMsg, value, len );
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( ancsMsgConfig, (uint8 *)&ancsMsg,
                                    FALSE, ancsAttrTbl, GATT_NUM_ATTRS( ancsAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    
    case DATASRC_ENABLER:
      if ( len == sizeof ( uint8 ) ) 
      {
        dataSrcEnabled = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DATASRC_MSG:
      if ( len <= MAX_LEN )
      {      
        dataSrcMsgLength = len;
        VOID osal_memcpy( dataSrcMsg, value, len );
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( dataSrcMsgConfig, (uint8 *)&dataSrcMsg,
                                    FALSE, ancsAttrTbl, GATT_NUM_ATTRS( ancsAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;     
     
    case CONTROL_POINT:
      if ( len <= MAX_LEN )
      {
        VOID osal_memcpy( controlPoint, value, len );
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
 * @fn      ANCS_GetParameter
 *
 * @brief   Get an ANCS Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t ANCS_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case ANCS_ENABLER:
      *((uint8*)value) = ancsEnabled;
      break;
      
    case ANCS_MSG:
      VOID osal_memcpy( value, ancsMsg, ancsMsgLength );
      break;
    
    case DATASRC_ENABLER:
      *((uint8*)value) = dataSrcEnabled;
      break;  
      
    case DATASRC_MSG:
      VOID osal_memcpy( value, dataSrcMsg, dataSrcMsgLength );
      break;
      
    case CONTROL_POINT:
      VOID osal_memcpy( value, controlPoint, controlPointLength );
      break;
    
    case CONTROL_POINT_LEN:
      *((uint8*)value) = controlPointLength;
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          ancs_ReadAttr
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
static uint8 ancs_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
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
      
      case ANCS_MSG_UUID:
        *pLen = ancsMsgLength;
        VOID osal_memcpy( pValue, pAttr->pValue, ancsMsgLength );
        break;
      
      case DATASRC_MSG_UUID:
        *pLen = dataSrcMsgLength;
        VOID osal_memcpy( pValue, pAttr->pValue, dataSrcMsgLength );
        break;
        
      case CONTROL_POINT_UUID:
        *pLen = controlPointLength;
        VOID osal_memcpy( pValue, pAttr->pValue, controlPointLength );
        break;
      
      case ANCS_ENABLER_UUID:
      case DATASRC_ENABLER_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
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
 * @fn      ancs_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle – connection message was received on
 * @param   pReq - pointer to request
 *
 * @return  Success or Failure
 */
static bStatus_t ancs_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notifyAPP = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case ANCS_ENABLER_UUID:
      case DATASRC_ENABLER_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > 1 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
          else if ( pValue[0] != FALSE && pValue[0] != TRUE )
            status = ATT_ERR_INVALID_VALUE;
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //status = ATT_ERR_INSUFFICIENT_ENCRYPT;
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          *pCurValue = pValue[0];
          switch (uuid)
          {
          case ANCS_ENABLER_UUID:
            notifyAPP = ANCS_ENABLER;
            break;
          case DATASRC_ENABLER_UUID:
            notifyAPP = DATASRC_ENABLER;
            break;
          default:
            break;
          }
        }
        break;
        
      case CONTROL_POINT_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > MAX_LEN )
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
          controlPointLength = len;
          notifyAPP = CONTROL_POINT;
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
  if ( (notifyAPP != 0xFF) && ancs_AppCBs && ancs_AppCBs->pfnANCSChange )
    ancs_AppCBs->pfnANCSChange(notifyAPP);  
  
  return ( status );
}

/*********************************************************************
 * @fn          ancs_HandleConnStatusCB
 *
 * @brief       ANCS Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void ancs_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, ancsMsgConfig );
      GATTServApp_InitCharCfg( connHandle, dataSrcMsgConfig );
    }
  }
}

// Write Control Point by Handler
void ANCSWriteCtrlPoint(uint16 connHandle, uint8 task_id, uint16 charHdl)
{
  attWriteReq_t wrreq;
  wrreq.handle = charHdl;
  wrreq.len = controlPointLength;
  VOID osal_memcpy( wrreq.value, controlPoint, wrreq.len );
  wrreq.sig = 0;
  wrreq.cmd = 0;
  GATT_WriteCharValue( connHandle, &wrreq, task_id ); 
}
/*********************************************************************
*********************************************************************/
