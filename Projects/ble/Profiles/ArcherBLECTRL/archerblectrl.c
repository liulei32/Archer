#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "archerblectrl.h"

#include "archerancs.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        39

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// BLECTRL Service UUID
CONST uint8 blectrlServiceUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_SERVICE_UUID), HI_UINT16(BLECTRL_SERVICE_UUID)
};

// BLECTRL Status UUID
CONST uint8 blectrlStatusUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_STATUS_UUID), HI_UINT16(BLECTRL_STATUS_UUID)
};

// BLECTRL Unclasssified Notification UUID
CONST uint8 blectrlUnclsNotifUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_UNCLS_NOTIF_UUID), HI_UINT16(BLECTRL_UNCLS_NOTIF_UUID)
};

// BLECTRL Discover Service UUID
CONST uint8 blectrlCommandUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_COMMAND_UUID), HI_UINT16(BLECTRL_COMMAND_UUID)
};

// BLECTRL Service UUID Length UUID
CONST uint8 blectrlServUUIDLenUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_SERV_UUID_LEN_UUID), HI_UINT16(BLECTRL_SERV_UUID_LEN_UUID)
};

// BLECTRL Service UUID UUID
CONST uint8 blectrlServUUIDUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_SERV_UUID_UUID), HI_UINT16(BLECTRL_SERV_UUID_UUID)
};

// BLECTRL Service Start Handler UUID
CONST uint8 blectrlServStartHdlUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_SERV_START_HDL_UUID), HI_UINT16(BLECTRL_SERV_START_HDL_UUID)
};

// BLECTRL Service End Handler UUID
CONST uint8 blectrlServEndHdlUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_SERV_END_HDL_UUID), HI_UINT16(BLECTRL_SERV_END_HDL_UUID)
};

// BLECTRL Characteristics UUID Length UUID
CONST uint8 blectrlCharUUIDLenUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_CHAR_UUID_LEN_UUID), HI_UINT16(BLECTRL_CHAR_UUID_LEN_UUID)
};

// BLECTRL Characteristics UUID UUID
CONST uint8 blectrlCharUUIDUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_CHAR_UUID_UUID), HI_UINT16(BLECTRL_CHAR_UUID_UUID)
};

// BLECTRL Characteristics Handler UUID
CONST uint8 blectrlCharHdlUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_CHAR_HDL_UUID), HI_UINT16(BLECTRL_CHAR_HDL_UUID)
};

// BLECTRL Data Length UUID
CONST uint8 blectrlDataLenUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_DATA_LEN_UUID), HI_UINT16(BLECTRL_DATA_LEN_UUID)
};

// BLECTRL Data UUID
CONST uint8 blectrlDataUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BLECTRL_DATA_UUID), HI_UINT16(BLECTRL_DATA_UUID)
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
static blectrlCBs_t *blectrl_AppCBs = NULL;

// BLE Ctrl Status Notification Counter
static uint8 blectrlStatusCount = 0;

/*********************************************************************
 * Profile Attributes - variables
 */

// BLECTRL Service attribute
static CONST gattAttrType_t blectrlService = { ATT_BT_UUID_SIZE, blectrlServiceUUID };

// Status Characteristic Properties
static uint8 blectrlStatusCharProps = GATT_PROP_NOTIFY;

// Status Characteristics
static uint8 blectrlStatus[3];

// Status Characteristic Configs
static gattCharCfg_t blectrlStatusConfig[GATT_MAX_NUM_CONN];

// Status Characteristic user descriptions
static uint8 blectrlStatusCharUserDesc[16] = "BLECTRL Status\0";

// Unclassified Notification Characteristic Properties
static uint8 blectrlUnclsNotifCharProps = GATT_PROP_NOTIFY;

// Unclassified Notification Characteristics
static uint8 blectrlUnclsNotif[MAX_LEN];
static uint8 unclsNotifLength;
// Unclassified Notification Characteristic Configs
static gattCharCfg_t blectrlUnclsNotifConfig[GATT_MAX_NUM_CONN];

// Unclassified Notification Characteristic user descriptions
static uint8 blectrlUnclsNotifCharUserDesc[21] = "BLECTRL Unclassified\0";

// Command Characteristic Properties
static uint8 blectrlCommandCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Command Characteristic Value
static uint8 blectrlCommand = 0;

// Command Characteristic user description
static uint8 blectrlCommandUserDesc[16] = "BLECTRL Command\0";

// Service_UUID_Length Characteristic Properties
static uint8 blectrlServUUIDLenCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Service_UUID_Length Characteristic Value
static uint8 blectrlServUUIDLen = ATT_UUID_SIZE;

// Service_UUID_Length Characteristic user description
static uint8 blectrlServUUIDLenUserDesc[22] = "BLECTRL Serv UUID Len\0";

// Service_UUID Characteristic Properties
static uint8 blectrlServUUIDCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Service_UUID Characteristic Value
static uint8 blectrlServUUID[16] = {APPLEANCS_SERVICE_UUID};

// Service_UUID Characteristic user description
static uint8 blectrlServUUIDUserDesc[18] = "BLECTRL Serv UUID\0";

// Discover_Service_Start_Handler Characteristic Properties
static uint8 blectrlServStartHdlCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Discover_Service_Start_Handler Characteristic Value
static uint16 blectrlServStartHdl = 0x0001;

// Discover_Service_Start_Handler Characteristic user description
static uint8 blectrlServStartHdlUserDesc[23] = "BLECTRL Serv Start Hdl\0";

// Discover_Service_End_Handler Characteristic Properties
static uint8 blectrlServEndHdlCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Discover_Service_End_Handler Characteristic Value
static uint16 blectrlServEndHdl = 0xFFFF;

// Discover_Service_End_Handler Characteristic user description
static uint8 blectrlServEndHdlUserDesc[21] = "BLECTRL Serv End Hdl\0";

// Characteristics_UUID_Length Characteristic Properties
static uint8 blectrlCharUUIDLenCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristics_UUID_Length Characteristic Value
static uint8 blectrlCharUUIDLen = ATT_UUID_SIZE;

// Characteristics_UUID_Length Characteristic user description
static uint8 blectrlCharUUIDLenUserDesc[22] = "BLECTRL Char UUID Len\0";

// Characteristics_UUID Characteristic Properties
static uint8 blectrlCharUUIDCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristics_UUID Characteristic Value
static uint8 blectrlCharUUID[16] = {APPLEANCS_NOTIFICATION_UUID};

// Characteristics_UUID Characteristic user description
static uint8 blectrlCharUUIDUserDesc[18] = "BLECTRL Char UUID\0";

// Characteristics_Handler Characteristic Properties
static uint8 blectrlCharHdlCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristics_Handler Characteristic Value
static uint16 blectrlCharHdl = 0;

// Characteristics_Handler Characteristic user description
static uint8 blectrlCharHdlUserDesc[21] = "BLECTRL Char Handler\0";

// Data_Length Characteristic Properties
static uint8 blectrlDataLenCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Data_Length Characteristic Value
static uint8 blectrlDataLen = 0;

// Data_Length Characteristic user description
static uint8 blectrlDataLenUserDesc[17] = "BLECTRL Data Len\0";

// Data Characteristic Properties
static uint8 blectrlDataCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Data Characteristic Value
static uint8 blectrlData[MAX_LEN];

// Data Characteristic user description
static uint8 blectrlDataUserDesc[13] = "BLECTRL Data\0";

/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t blectrlAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // BLECTRL Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                   /* permissions */
    0,                                  /* handle */
    (uint8 *)&blectrlService                /* pValue */
  },
      
    // Status Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlStatusCharProps 
    },
      // Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlStatusUUID },
        0, 
        0, 
        blectrlStatus
      },
      
      // Status Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)blectrlStatusConfig 
      },

      // Status Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        blectrlStatusCharUserDesc
      },  
        
    // Unclassified Notification Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlUnclsNotifCharProps 
    },
      // Unclassified Notification Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlUnclsNotifUUID },
        0, 
        0, 
        blectrlUnclsNotif
      },
      
      // Unclassified Notification Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)blectrlUnclsNotifConfig 
      },

      // Unclassified Notification Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        blectrlUnclsNotifCharUserDesc
      },  
  
    // Discover_Service Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlCommandCharProps
    },

      // Discover_Service Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlCommandUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        &blectrlCommand 
      },

      // Discover_Service User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&blectrlCommandUserDesc
      },
            
    // Service_UUID_Len Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlServUUIDLenCharProps
    },

      // Service_UUID_Len Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlServUUIDLenUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        &blectrlServUUIDLen 
      },

      // Service_UUID_Len User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&blectrlServUUIDLenUserDesc
      },
            
    // Service_UUID Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlServUUIDCharProps
    },

      // Service_UUID Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlServUUIDUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        blectrlServUUID 
      },

      // Service_UUID User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&blectrlServUUIDUserDesc
      },
      
    // Discover_Service_Start_Handler Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlServStartHdlCharProps
    },

      // Discover_Service_Start_Handler Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlServStartHdlUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        (uint8 *)&blectrlServStartHdl 
      },

      // Discover_Service_Start_Handler User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&blectrlServStartHdlUserDesc
      },
          
    // Discover_Service_End_Handler Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlServEndHdlCharProps
    },

      // Discover_Service_End_Handler Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlServEndHdlUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        (uint8 *)&blectrlServEndHdl 
      },

      // Discover_Service_End_Handler User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&blectrlServEndHdlUserDesc
      },
                  
    // Characteristics_UUID_Len Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlCharUUIDLenCharProps
    },

      // Characteristics_UUID_Len Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlCharUUIDLenUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        &blectrlCharUUIDLen 
      },

      // Characteristics_UUID_Len User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&blectrlCharUUIDLenUserDesc
      },
            
    // Characteristics_UUID Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlCharUUIDCharProps
    },

      // Characteristics_UUID Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlCharUUIDUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        blectrlCharUUID 
      },

      // Characteristics_UUID User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&blectrlCharUUIDUserDesc
      },
            
    // Characteristics_Handler Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlCharHdlCharProps
    },

      // Characteristics_UUID Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlCharHdlUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        (uint8*)&blectrlCharHdl 
      },

      // Characteristics_UUID User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&blectrlCharHdlUserDesc
      },
      
    // Data_Length Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlDataLenCharProps
    },

      // Data_Length Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlDataLenUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        &blectrlDataLen 
      },

      // Data_Length User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&blectrlDataLenUserDesc
      },
      
    // Data Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &blectrlDataCharProps
    },

      // Data Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, blectrlDataUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        blectrlData 
      },

      // Data User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        (uint8*)&blectrlDataUserDesc
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 blectrl_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                               uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t blectrl_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset );

static void blectrl_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
//  BLECTRL Service Callbacks
CONST gattServiceCBs_t  blectrlCBs =
{
  blectrl_ReadAttrCB,  // Read callback function pointer
  blectrl_WriteAttrCB, // Write callback function pointer
  NULL               // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      BLECTRL_AddService
 *
 * @brief   Initializes the BLECTRL service by
 *          registering GATT attributes with the GATT server. Only
 *          call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t BLECTRL_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, blectrlStatusConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, blectrlUnclsNotifConfig );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( blectrl_HandleConnStatusCB );  

  if ( services & BLECTRL_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( blectrlAttrTbl, 
                                          GATT_NUM_ATTRS( blectrlAttrTbl ),
                                          &blectrlCBs );
  }

  return ( status );
}

/*********************************************************************
 * @fn      BLECTRL_RegisterAppCBs
 *
 * @brief   Does the profile initialization.  Only call this function
 *          once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t BLECTRL_RegisterAppCBs( blectrlCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    blectrl_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


/*********************************************************************
 * @fn      BLECTRL_SetParameter
 *
 * @brief   Set an BLECTRL Profile parameter.
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
bStatus_t BLECTRL_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case BLECTRL_STATUS:
      if ( len == 3 * sizeof ( int8 ) ) 
      {  
        VOID osal_memcpy( blectrlStatus, value, len );
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( blectrlStatusConfig, blectrlStatus,
                                    FALSE, blectrlAttrTbl, GATT_NUM_ATTRS( blectrlAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break; 
        
    case BLECTRL_UNCLS_NOTIF:
      if ( len <= MAX_LEN ) 
      {      
        unclsNotifLength = len;
        VOID osal_memcpy( blectrlUnclsNotif, value, len );
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( blectrlUnclsNotifConfig, blectrlUnclsNotif,
                                    FALSE, blectrlAttrTbl, GATT_NUM_ATTRS( blectrlAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;  
      
    case BLECTRL_COMMAND:
      if ( len == sizeof ( uint8 ) ) 
        blectrlCommand = *((uint8*)value);
      else
        ret = bleInvalidRange;
      break; 
      
    case BLECTRL_SERV_UUID_LEN:
      if ( len == sizeof ( uint8 ) ) 
        blectrlServUUIDLen = *((uint8*)value);
      else
        ret = bleInvalidRange;
      break;   
      
    case BLECTRL_SERV_UUID:
      if ( len <= 16 * sizeof ( uint8 ) ) 
        VOID osal_memcpy( blectrlServUUID, value, len );
      else
        ret = bleInvalidRange;
      break;
          
    case BLECTRL_SERV_START_HDL:
      if ( len == sizeof ( uint16 ) ) 
        blectrlServStartHdl = *((uint16*)value);
      else
        ret = bleInvalidRange;
      break; 
    
    case BLECTRL_SERV_END_HDL:
      if ( len == sizeof ( uint16 ) ) 
        blectrlServEndHdl = *((uint16*)value);
      else
        ret = bleInvalidRange;
      break; 
      
    case BLECTRL_CHAR_UUID_LEN:
      if ( len == sizeof ( uint8 ) ) 
        blectrlCharUUIDLen = *((uint8*)value);
      else
        ret = bleInvalidRange;
      break;   
      
    case BLECTRL_CHAR_UUID:
      if ( len <= 16 * sizeof ( uint8 ) ) 
        VOID osal_memcpy( blectrlCharUUID, value, len );
      else
        ret = bleInvalidRange;
      break;
   
    case BLECTRL_CHAR_HDL:
      if ( len == sizeof ( uint16 ) ) 
        blectrlCharHdl = *((uint16*)value);
      else
        ret = bleInvalidRange;
      break;
      
    case BLECTRL_DATA_LEN:
      if ( len == sizeof ( uint8 ) ) 
        blectrlDataLen = *((uint8*)value);
      else
        ret = bleInvalidRange;
      break;
      
    case BLECTRL_DATA:
      if ( len <= 16 * sizeof ( uint8 ) ) 
        VOID osal_memcpy( blectrlData, value, len );
      else
        ret = bleInvalidRange;
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      BLECTRL_GetParameter
 *
 * @brief   Get an BLECTRL Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t BLECTRL_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case BLECTRL_STATUS:
      VOID osal_memcpy( value, blectrlStatus, 3 );
      break;
         
    case BLECTRL_UNCLS_NOTIF:
      VOID osal_memcpy( value, blectrlUnclsNotif, unclsNotifLength );
      break;
      
    case BLECTRL_COMMAND:
      *((uint8*)value) = blectrlCommand;
      break;
      
    case BLECTRL_SERV_UUID_LEN:
      *((uint8*)value) = blectrlServUUIDLen;
      break;
      
    case BLECTRL_SERV_UUID:
      VOID osal_memcpy( value, blectrlServUUID, 16 );
      break;
           
    case BLECTRL_SERV_START_HDL:
      *((uint16*)value) = blectrlServStartHdl;
      break;
      
    case BLECTRL_SERV_END_HDL:
      *((uint16*)value) = blectrlServEndHdl;
      break;
      
    case BLECTRL_CHAR_UUID_LEN:
      *((uint8*)value) = blectrlCharUUIDLen;
      break;
      
    case BLECTRL_CHAR_UUID:
      VOID osal_memcpy( value, blectrlCharUUID, 16 );
      break;
      
    case BLECTRL_CHAR_HDL:
      *((uint16*)value) = blectrlCharHdl;
      break;
      
    case BLECTRL_DATA_LEN:
      *((uint8*)value) = blectrlDataLen;
      break;
      
    case BLECTRL_DATA:
      VOID osal_memcpy( value, blectrlData, 16 );
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          blectrl_ReadAttr
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
static uint8 blectrl_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
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
      case BLECTRL_STATUS_UUID:
        *pLen = 3;
        VOID osal_memcpy( pValue, pAttr->pValue, 16 );
        break;
      case BLECTRL_UNCLS_NOTIF_UUID:
        *pLen = unclsNotifLength;
        VOID osal_memcpy( pValue, pAttr->pValue, unclsNotifLength );
        break;  
      case BLECTRL_COMMAND_UUID:
      case BLECTRL_SERV_UUID_LEN_UUID:
      case BLECTRL_CHAR_UUID_LEN_UUID:
      case BLECTRL_DATA_LEN_UUID:  
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
        
      case BLECTRL_SERV_UUID_UUID:
      case BLECTRL_CHAR_UUID_UUID:
      case BLECTRL_DATA_UUID:  
        *pLen = 16;
        VOID osal_memcpy( pValue, pAttr->pValue, 16 );
        break;
      
      case BLECTRL_SERV_START_HDL_UUID: 
      case BLECTRL_SERV_END_HDL_UUID:   
      case BLECTRL_CHAR_HDL_UUID:   
        *pLen = 2;
        pValue[0] = LO_UINT16( *((uint16 *)pAttr->pValue) );
        pValue[1] = HI_UINT16( *((uint16 *)pAttr->pValue) );
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
 * @fn      blectrl_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle – connection message was received on
 * @param   pReq - pointer to request
 *
 * @return  Success or Failure
 */
static bStatus_t blectrl_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notifyAPP = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case BLECTRL_COMMAND_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > 1 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
          else if ( pValue[0] != BLECTRL_CMD_DISCOVER_SERVICE
                   && pValue[0] != BLECTRL_CMD_DISCOVER_CHARACTERISTICS 
                   && pValue[0] != BLECTRL_CMD_READ_CHARS_VALUE
                   && pValue[0] != BLECTRL_CMD_WRITE_CHARS_VALUE
                   && pValue[0] != BLECTRL_CMD_SUBSCRIBE_CHARACTERISTICS
                   && pValue[0] != BLECTRL_CMD_UNSUBSCRIBE_CHARACTERISTICS)
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
           notifyAPP = BLECTRL_COMMAND;
        }
        break;
        
      case BLECTRL_SERV_UUID_LEN_UUID:
      case BLECTRL_CHAR_UUID_LEN_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > 1 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
          else if ( pValue[0] != ATT_UUID_SIZE && pValue[0] != ATT_BT_UUID_SIZE )
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
        }
        break;
      
      case BLECTRL_DATA_LEN_UUID:  
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > 1 )
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
          *pCurValue = pValue[0];
        }
        break;
        
      case BLECTRL_SERV_UUID_UUID:
      case BLECTRL_CHAR_UUID_UUID:
      case BLECTRL_DATA_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > 16 )
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
        break;     
        
      case BLECTRL_SERV_START_HDL_UUID:
      case BLECTRL_SERV_END_HDL_UUID:
      case BLECTRL_CHAR_HDL_UUID:   
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != 2 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint16 *pCurValue = (uint16 *)pAttr->pValue;
          *pCurValue = BUILD_UINT16(pValue[0],pValue[1]);
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
  if ( (notifyAPP != 0xFF ) && blectrl_AppCBs && blectrl_AppCBs->pfnBLECTRLChange )
    blectrl_AppCBs->pfnBLECTRLChange(notifyAPP);  
  
  return ( status );
}

/*********************************************************************
 * @fn          blectrl_HandleConnStatusCB
 *
 * @brief       BLECTRL Service link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void blectrl_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, blectrlStatusConfig );
      GATTServApp_InitCharCfg( connHandle, blectrlUnclsNotifConfig );
    }
  }
}

// APP reports status in this function
void BLECtrlStatusReport( uint8 status, uint8 moreInfo)
{
  uint8 dataToWrite[3];
  blectrlStatusCount++;
  dataToWrite[0] = blectrlStatusCount;
  dataToWrite[1] = status;
  dataToWrite[2] = moreInfo;
  BLECTRL_SetParameter( BLECTRL_STATUS, 3, dataToWrite );
}

// Discover Service
void BLECtrlDiscoverService(uint16 connHandle, uint8 task_id)
{
  GATT_DiscPrimaryServiceByUUID( connHandle,
                                 blectrlServUUID,
                                 blectrlServUUIDLen,
                                 task_id ); // TODO Verify uuid_len == 2
}

// Discover Chars
void BLECtrlDiscoverChars(uint16 connHandle, uint8 task_id)
{
  attReadByTypeReq_t req;
  req.startHandle = blectrlServStartHdl;
  req.endHandle = blectrlServEndHdl;
  req.type.len = blectrlCharUUIDLen;
  VOID osal_memcpy( req.type.uuid, blectrlCharUUID, blectrlCharUUIDLen );    
  GATT_DiscCharsByUUID(connHandle, &req, task_id); // TODO verify uuid_len == 2
}

// Read by Handler
void BLECtrlReadByHdl(uint16 connHandle, uint8 task_id)
{
  attReadReq_t rdreq;
  rdreq.handle = blectrlCharHdl;
  GATT_ReadCharValue( connHandle, &rdreq, task_id );
}

// Write by Handler
void BLECtrlWriteByHdl(uint16 connHandle, uint8 task_id)
{
  attWriteReq_t wrreq;
  wrreq.handle = blectrlCharHdl;
  wrreq.len = blectrlDataLen;
  VOID osal_memcpy( wrreq.value, blectrlData, blectrlDataLen );
  wrreq.sig = 0;
  wrreq.cmd = 0;
  GATT_WriteCharValue( connHandle, &wrreq, task_id ); 
}

// Subscribe/Unsubscribe by Handler
void BLECtrlSubscribeByHdl(uint16 connHandle, uint8 task_id, uint16 charHdl, uint8 subscribe)
{
  attWriteReq_t wrreq;
  wrreq.handle = charHdl + 1;
  wrreq.len = 2;
  wrreq.value[0] = subscribe;
  wrreq.value[1] = 0x00;
  wrreq.sig = 0;
  wrreq.cmd = 0;
  GATT_WriteCharValue( connHandle, &wrreq, task_id ); 
}

// Discover Chars by UUID
void BLECtrlDiscoverCharsByUUID16(uint16 connHandle, uint8 task_id, uint8* uuid)
{
  attReadByTypeReq_t req;
  req.startHandle = 0x0001;
  req.endHandle = 0xFFFF;
  req.type.len = 16;
  VOID osal_memcpy(req.type.uuid, uuid, 16);
  GATT_DiscCharsByUUID(connHandle, &req, task_id); // TODO verify uuid_len == 2
}
/*********************************************************************
*********************************************************************/
