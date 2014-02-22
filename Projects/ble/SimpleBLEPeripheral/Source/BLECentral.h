#include "gatt.h"

void simpleBLECentralSetConnHandle(uint16 hdl);
void simpleBLECentralSetTaskID(uint8 task_id);
void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
void simpleBLECentralExecuteCommand( void );
void simpleBLECentralSubscribeANCS(uint8 value);
void simpleBLECentralSubscribeDataSrc(uint8 value);