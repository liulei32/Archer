#include "BLECentral.h"

#include "gatt.h"
#include "hal_archerled.h"
#include "archerblectrl.h"
#include "archergeneral.h"
#include "archerancs.h"

#include "simpleBLEPeripheral.h"


// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           500

static uint8 simpleBLEPeripheral_TaskID;

// Command Combo
static uint8 commandCombo = BLECTRL_CMD_IDLE;

// ANCS Profile Parameters
static uint8 ancsUUID[16] = {APPLEANCS_NOTIFICATION_UUID};
static uint8 ANCSEnabler = FALSE;
static uint16 ancsMsgHdl = 0;

static uint8 dataSrcUUID[16] = {APPLEDATASRC_NOTIFICATION_UUID};
static uint8 dataSrcEnabler = FALSE;
static uint16 dataSrcHdl = 0;

static uint8 controlPointUUID[16] = {APPLECTRLPT_UUID};
static uint16 controlPointHdl = 0;

// Discovery state// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 simpleBLESvcStartHdl = 0;
static uint16 simpleBLESvcEndHdl = 0;
// Discovered characteristic handle
static uint16 simpleBLECharHdl = 0;
// BLE connection handle
static uint16 connHandle;




void simpleBLECentralSetConnHandle(uint16 hdl)
{
  connHandle = hdl;
}

void simpleBLECentralSetTaskID(uint8 task_id)
{
  simpleBLEPeripheral_TaskID = task_id;
}

/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  /*if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }*/
  
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP ) // Read fails
      BLECtrlStatusReport( BLECTRL_READ_CHARACTERISTICS_ERROR_RSP, pMsg->msg.errorRsp.errCode);
    else // Read successes
    {
      HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK );
      BLECtrlStatusReport( BLECTRL_READ_CHARACTERISTICS_RSP, 0);
      
      BLECTRL_SetParameter( BLECTRL_DATA_LEN, sizeof(uint8), &(pMsg->msg.readRsp.len) );
      BLECTRL_SetParameter( BLECTRL_DATA, pMsg->msg.readRsp.len , pMsg->msg.readRsp.value );
    }
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP) // Write fails
    {
      BLECtrlStatusReport( BLECTRL_WRITE_CHARACTERISTICS_ERROR_RSP, pMsg->msg.errorRsp.errCode);
      if (commandCombo == BLECTRL_CMD_SUBS_UNSU_ANCS)
      {
        commandCombo = BLECTRL_CMD_IDLE;
      }
    }
    else // Write Successes
    {
      HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK );
      BLECtrlStatusReport( BLECTRL_WRITE_CHARACTERISTICS_RSP, 0);
      // Return ANCS subscription states
      if (commandCombo == BLECTRL_CMD_SUBS_UNSU_ANCS)
      {
        commandCombo = BLECTRL_CMD_IDLE;
        General_SetParameter( ANCS_STATE, sizeof(uint8),  &ANCSEnabler);
      }
    }
  }
  else if ( pMsg->method == ATT_HANDLE_VALUE_NOTI )
  {
      HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK );
      if (pMsg->msg.handleValueNoti.handle == ancsMsgHdl)
        ANCS_SetParameter( ANCS_MSG, pMsg->msg.handleValueNoti.len, pMsg->msg.handleValueNoti.value );
      else if (pMsg->msg.handleValueNoti.handle == dataSrcHdl)
        ANCS_SetParameter( DATASRC_MSG, pMsg->msg.handleValueNoti.len, pMsg->msg.handleValueNoti.value );
      else
        BLECTRL_SetParameter( BLECTRL_UNCLS_NOTIF, pMsg->msg.handleValueNoti.len, pMsg->msg.handleValueNoti.value );
      // TODO add one more notification point for unanticipated notification
  }
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )
  {
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP)
    {
      if (pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        BLECtrlStatusReport( BLECTRL_DISCOVER_SERVICE_RSP, pMsg->msg.findByTypeValueRsp.numInfo);
        
        HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK );
        simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        BLECTRL_SetParameter( BLECTRL_SERV_START_HDL, sizeof(uint16), &simpleBLESvcStartHdl );
        simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
        BLECTRL_SetParameter( BLECTRL_SERV_END_HDL, sizeof(uint16), &simpleBLESvcEndHdl );
      }
    }
    // If procedure complete
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
           pMsg->hdr.status == bleProcedureComplete )
    {
        BLECtrlStatusReport( BLECTRL_DISCOVER_SERVICE_RSP_COMPLETE, 0);
    }
    if (pMsg->method == ATT_ERROR_RSP) 
    {
        BLECtrlStatusReport( BLECTRL_DISCOVER_SERVICE_ERROR_RSP, pMsg->msg.errorRsp.errCode);
    }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
      BLECtrlStatusReport( BLECTRL_DISCOVER_CHARACTERISTICS_RSP, pMsg->msg.readByTypeRsp.numPairs);
      
      HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK );
      
      simpleBLECharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[3],
                                       pMsg->msg.readByTypeRsp.dataList[4] );
      //combo command
      if (commandCombo == BLECTRL_CMD_SUBSCRIBE_CHARACTERISTICS
        || commandCombo == BLECTRL_CMD_UNSUBSCRIBE_CHARACTERISTICS)
      {
        simpleBLECharHdl = simpleBLECharHdl + 1;
        BLECTRL_SetParameter( BLECTRL_CHAR_HDL, sizeof(uint16), &simpleBLECharHdl );
        uint8 data_len = 2;
        BLECTRL_SetParameter( BLECTRL_DATA_LEN, sizeof(uint8), &data_len );
        uint8 data[2] = {0x00, 0x00};
        if (commandCombo == BLECTRL_CMD_SUBSCRIBE_CHARACTERISTICS)
          data[0] = 0x01;
        BLECTRL_SetParameter( BLECTRL_DATA, 2*sizeof(uint8), data );
        uint8 next_command = BLECTRL_CMD_WRITE_CHARS_VALUE;
        BLECTRL_SetParameter( BLECTRL_COMMAND, sizeof(uint8), &next_command );
        osal_start_timerEx( simpleBLEPeripheral_TaskID, EXECUTE_COMMAND_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
      }
      else if (commandCombo == BLECTRL_CMD_SUBS_UNSU_ANCS)
      {
        ancsMsgHdl = simpleBLECharHdl;
        osal_start_timerEx( simpleBLEPeripheral_TaskID, EXECUTE_COMMAND_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
      }
      else if (commandCombo == BLECTRL_CMD_SUBS_UNSU_DATASRC)
      {
        dataSrcHdl = simpleBLECharHdl;
        osal_start_timerEx( simpleBLEPeripheral_TaskID, EXECUTE_COMMAND_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
      }
      else if (commandCombo == BLECTRL_CMD_WRITE_CONTROL_POINT)
      {
        controlPointHdl = simpleBLECharHdl;
        osal_start_timerEx( simpleBLEPeripheral_TaskID, EXECUTE_COMMAND_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
      }
      else // not in any of combo command
      {
        uint8 data_len = 5;
        BLECTRL_SetParameter( BLECTRL_DATA_LEN, sizeof(uint8), &data_len );
        BLECTRL_SetParameter( BLECTRL_DATA, 5*sizeof(uint8), pMsg->msg.readByTypeRsp.dataList );
        BLECTRL_SetParameter( BLECTRL_CHAR_HDL, sizeof(uint16), &simpleBLECharHdl );
      }
    }
    simpleBLEDiscState = BLE_DISC_STATE_IDLE;
  }
}


/*********************************************************************
 * @fn      simpleBLECentralExecuteCommand
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
void simpleBLECentralExecuteCommand( void )
{
  uint8 command;
  uint8 next_command = BLECTRL_CMD_IDLE;
  BLECTRL_GetParameter( BLECTRL_COMMAND, &command );
  switch (command)
  {
  case BLECTRL_CMD_DISCOVER_SERVICE:
    // Initialize cached handles
    simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;
    BLECTRL_SetParameter( BLECTRL_SERV_START_HDL, sizeof(uint16), &simpleBLESvcStartHdl );
    BLECTRL_SetParameter( BLECTRL_SERV_END_HDL, sizeof(uint16), &simpleBLESvcEndHdl );
    
    // Discovery service
    simpleBLEDiscState = BLE_DISC_STATE_SVC;
    BLECtrlDiscoverService(connHandle, simpleBLEPeripheral_TaskID);
    break;
  case BLECTRL_CMD_DISCOVER_CHARACTERISTICS:
    BLECTRL_GetParameter( BLECTRL_SERV_START_HDL, &simpleBLESvcStartHdl );
    if (simpleBLESvcStartHdl != 0 )
    {
      simpleBLEDiscState = BLE_DISC_STATE_CHAR;
      BLECtrlDiscoverChars(connHandle, simpleBLEPeripheral_TaskID);
    }
    break;
  case BLECTRL_CMD_READ_CHARS_VALUE:
    BLECtrlReadByHdl(connHandle, simpleBLEPeripheral_TaskID);
    break;
  case BLECTRL_CMD_WRITE_CHARS_VALUE:
    commandCombo = BLECTRL_CMD_IDLE;
    BLECtrlWriteByHdl(connHandle, simpleBLEPeripheral_TaskID);   
    break;
  case BLECTRL_CMD_SUBSCRIBE_CHARACTERISTICS:
  case BLECTRL_CMD_UNSUBSCRIBE_CHARACTERISTICS:
    commandCombo = command;
    next_command = BLECTRL_CMD_DISCOVER_CHARACTERISTICS;
    osal_start_timerEx( simpleBLEPeripheral_TaskID, EXECUTE_COMMAND_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
    break;
  case BLECTRL_CMD_SUBS_UNSU_ANCS:
    if (ancsMsgHdl != 0) // ancsMsgHdl is known
    {
      commandCombo = BLECTRL_CMD_SUBS_UNSU_ANCS;
      BLECtrlSubscribeByHdl(connHandle, simpleBLEPeripheral_TaskID, ancsMsgHdl, ANCSEnabler);
    }
    else // ancsMsgHdl is unknown, discover it first
    {
      commandCombo = BLECTRL_CMD_SUBS_UNSU_ANCS;
      next_command = BLECTRL_CMD_SUBS_UNSU_ANCS;
      simpleBLEDiscState = BLE_DISC_STATE_CHAR;
      BLECtrlDiscoverCharsByUUID16(connHandle, simpleBLEPeripheral_TaskID, ancsUUID);
    }
    break;
  case BLECTRL_CMD_SUBS_UNSU_DATASRC:
    if (dataSrcHdl != 0) // dataSrcHdl is known
    {
      commandCombo = BLECTRL_CMD_IDLE;
      BLECtrlSubscribeByHdl(connHandle, simpleBLEPeripheral_TaskID, dataSrcHdl, dataSrcEnabler);
    }
    else // dataSrcHdl is unknown, discover it first
    {
      commandCombo = BLECTRL_CMD_SUBS_UNSU_DATASRC;
      next_command = BLECTRL_CMD_SUBS_UNSU_DATASRC;
      simpleBLEDiscState = BLE_DISC_STATE_CHAR;
      BLECtrlDiscoverCharsByUUID16(connHandle, simpleBLEPeripheral_TaskID, dataSrcUUID);
    }
    break;
  case BLECTRL_CMD_WRITE_CONTROL_POINT:
    if (controlPointHdl != 0) // controlPointHdl is known
    {
      commandCombo = BLECTRL_CMD_IDLE;
      ANCSWriteCtrlPoint(connHandle, simpleBLEPeripheral_TaskID, controlPointHdl);
    }
    else // controlPointHdl is unknown, discover it first
    {
      commandCombo = BLECTRL_CMD_WRITE_CONTROL_POINT;
      next_command = BLECTRL_CMD_WRITE_CONTROL_POINT;
      simpleBLEDiscState = BLE_DISC_STATE_CHAR;
      BLECtrlDiscoverCharsByUUID16(connHandle, simpleBLEPeripheral_TaskID, controlPointUUID);
    }
    break;
  default:
    break;
  }
  BLECTRL_SetParameter( BLECTRL_COMMAND, sizeof(uint8), &next_command );
}

void simpleBLECentralSubscribeANCS(uint8 value)
{
  ANCSEnabler = value;
  uint8 writeValue = BLECTRL_CMD_SUBS_UNSU_ANCS;
  BLECTRL_SetParameter( BLECTRL_COMMAND, sizeof(uint8), &writeValue );
  osal_start_timerEx( simpleBLEPeripheral_TaskID, EXECUTE_COMMAND_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
}

void simpleBLECentralSubscribeDataSrc(uint8 value)
{
  dataSrcEnabler = value;
  uint8 writeValue = BLECTRL_CMD_SUBS_UNSU_DATASRC;
  BLECTRL_SetParameter( BLECTRL_COMMAND, sizeof(uint8), &writeValue );
  osal_start_timerEx( simpleBLEPeripheral_TaskID, EXECUTE_COMMAND_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
}
