/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_archerwatch.h"
#include "osal.h"
#include "hal_board.h"
   
#include "hal_archerled.h"

/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/


/***************************************************************************************************
 *                                           GLOBAL VARIABLES
 ***************************************************************************************************/

static uint8 Hour;
static uint8 Minute;
static uint8 Second;

/***************************************************************************************************
 *                                            LOCAL FUNCTION
 ***************************************************************************************************/
void ArcherClockInit(void);
void ArcherClockUpdate(void);

/***************************************************************************************************
 *                                            FUNCTIONS - API
 ***************************************************************************************************/

void ArcherClockInit (void)
{
  Hour = 0;
  Minute = 0;
  Second = 0;
  osal_start_timerEx(Hal_TaskID, HAL_CLOCK_EVENT, 1000);   // Schedule event
}

void ArcherClockSet (uint8 hour, uint8 minute, uint8 second)
{
  Hour = hour;
  Minute = minute;
  Second = second;
  osal_start_timerEx(Hal_TaskID, HAL_CLOCK_EVENT, 1000);   // Schedule event
}

void ArcherClockUpdate (void)
{
  osal_start_timerEx(Hal_TaskID, HAL_CLOCK_EVENT, 1000);   // Schedule event
  if (Second == 59)
  {
      Second = 0;
      if (Minute == 59)
      {
          Minute = 0;
          if (Hour == 23)
              Hour = 0;
          else
              Hour++;
      }
      else
          Minute++;
  }
  else
      Second++;
  HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK );
}

