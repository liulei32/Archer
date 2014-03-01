/**************************************************************************************************
  Filename:       hal_led.c
  Revised:        $Date: 2012-10-26 14:09:08 -0700 (Fri, 26 Oct 2012) $
  Revision:       $Revision: 31932 $

  Description:    This file contains the interface to the HAL LED Service.


  Copyright 2006-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_archerled.h"
#include "osal.h"
#include "hal_board.h"

#include "hal_i2c.h"

/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/
#define LEDCLR_OFF      0
#define LEDCLR_GREEN    1
#define LEDCLR_RED      2
#define LEDCLR_YELLOW   3

// [7:0]
#define BRIGHTNESS_9532 0xff
// [7:2]
#define BRIGHTNESS_9632_IND 0xfc
// [7:4]
#define BRIGHTNESS_9632_GRP 0xf0

#define I2CCLOCK i2cClock_123KHZ

/* LED control structure */
typedef struct {
  uint8 mode;       /* Operation mode */
  uint8 left;       /* Blink cycles left */
  uint8 onPct;      /* On cycle percentage */
  uint16 time;      /* On/off cycle time (msec) */
  uint32 next;      /* Time for next change */
} HalLedControl_t;

typedef struct
{
  HalLedControl_t HalLedControlTable[HAL_LED_DEFAULT_MAX_LEDS];
  uint8           sleepActive;
} HalLedStatus_t;

// Archer Features
typedef struct {
  uint8 color;
  uint8 onLeft;
  uint8 offLeft;
  uint8 onTime;
  uint8 offTime;
  uint8 next;
} ArcherLedControl_t;

static ArcherLedControl_t ledCtrl[NUM_LED];



/***************************************************************************************************
 *                                           GLOBAL VARIABLES
 ***************************************************************************************************/

static uint8 HalLedState;              // LED state at last set/clr/blink update

#if HAL_LED == TRUE
static uint8 HalSleepLedState;         // LED state at last set/clr/blink update
static uint8 preBlinkState;            // Original State before going to blink mode
                                       // bit 0, 1, 2, 3 represent led 0, 1, 2, 3
#endif

#ifdef BLINK_LEDS
  static HalLedStatus_t HalLedStatusControl;
#endif

static uint32 archer9532 = 0; // [upperLED15->0, lowerLED15->0]
static uint32 lastArcher9532 = 0;
static uint8 archerHourTable[12] = {23, 20, 18, 16, 12, 10, 7, 5, 2, 0, 30, 27};
static uint8 archerMinTable[16] = {22, 21, 19, 17, 14, 11, 9, 8, 6, 4, 3, 1, 31, 29, 28, 26};

/***************************************************************************************************
 *                                            LOCAL FUNCTION
 ***************************************************************************************************/
void ArcherLed1Update(void);
void ArcherLed2Update(void);
void ArcherLed3Update(void);
#if (HAL_LED == TRUE)
void HalLedUpdate (void);
void HalLedOnOff (uint8 leds, uint8 mode);
#endif /* HAL_LED */

/***************************************************************************************************
 *                                            FUNCTIONS - API
 ***************************************************************************************************/

/***************************************************************************************************
 * @fn      HalLedInit
 *
 * @brief   Initialize LED Service
 *
 * @param   init - pointer to void that contains the initialized value
 *
 * @return  None
 ***************************************************************************************************/
void HalLedInit (void)
{
#if (HAL_LED == TRUE)
  HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF);  // Initialize all LEDs to OFF.

  // Set LED GPIOs to outputs.
  LED1_DDR |= LED1_BV;
#if (!defined HAL_PA_LNA && !defined HAL_PA_LNA_CC2590)
  LED2_DDR |= LED2_BV;
//#if (!defined CC2540_MINIDK && !defined HAL_BOARD_CC2540USB)
  LED3_DDR |= LED3_BV;
//#endif
#endif
#if defined BLINK_LEDS
  HalLedStatusControl.sleepActive = FALSE;  // Initialize sleepActive to FALSE.
#endif
#endif
}

void ArcherLedSet(uint8 ledIndex, uint8 color, uint8 numBlinks, uint8 offsetTime, uint8 onTime, uint8 offTime)
{
  /*
  PERCFG &= ~0x10;             // Timer 4 Alternate location 1
  P1DIR |= 0x01;               // P1_0 = output
  P1SEL |= 0x01;               // P1_0 is peripheral function
  P2SEL |= 0x10;               // Timer 4 has priority on Port 1
  
  //T4CTL &= ~0xE0;              // Timer 4 tick frequency
  T4CTL |= 0xE0;              // Timer 4 tick frequency/128
  T4CTL &= ~0x10;             // Stop timer 4 (if it was running)
  T4CTL |= 0x04;              // Clear timer 4
  T4CTL &= ~0x08;             // Disable Timer 4 overflow interrupts
  T4CTL &= ~0x03;              // Timer 4 mode = 0 Free Run
  
  T4CCTL0 &= ~0x40;           // Disable channel 0 interrupts
  T4CCTL0 |= 0x04;            // Ch0 mode = compare
  T4CCTL0 |= 0x20;            // Ch0 output compare mode = clear output on compare, set on 0
  
  T4CC0 = offTime;
  
  T4CTL |= 0x10;
  */
  
  
  if (ledIndex<=(NUM_LED-1)) // ledIndex is valid
  {
    ledCtrl[ledIndex].color = color;
    ledCtrl[ledIndex].onTime = onTime;
    ledCtrl[ledIndex].offTime = offTime;
    ledCtrl[ledIndex].next = color;
    if (color == LEDCLR_OFF)    // if turn OFF LED, don't care left time
    {
      ledCtrl[ledIndex].onLeft = 0;
      ledCtrl[ledIndex].offLeft = 0;
    }
    else
    {
      ledCtrl[ledIndex].onLeft = numBlinks - 1;
      ledCtrl[ledIndex].offLeft = numBlinks;
    }
      
    switch (ledIndex)
    {
      case 0:
      osal_stop_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT1);
      osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT1, (uint32)offsetTime*100);   // Schedule event
      break;
      
      case 1:
      osal_stop_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT2);
      osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT2, (uint32)offsetTime*100);   // Schedule event
      break;
      
      case 2:
      osal_stop_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT3);
      osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT3, (uint32)offsetTime*100);   // Schedule event
      break;
    }
  }
}

void ArcherLed1Update (void)
{
  if (ledCtrl[0].next == 0)
    HAL_TURN_OFF_LED1();
  else 
    HAL_TURN_ON_LED1();
  
  if (ledCtrl[0].next == 0) // Current command is to turn OFF
  {
    if (ledCtrl[0].onLeft != 0) // There is flashes left to turn it ON
    {
      ledCtrl[0].onLeft = ledCtrl[0].onLeft - 1;
      ledCtrl[0].next = ledCtrl[0].color; // next command is to turn ON
      osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT1, (uint32)ledCtrl[0].offTime*100);   /* Schedule event */
    }
  }
  else // Current command is to turn ON
  {
    if (ledCtrl[0].offLeft != 0) // There is flashes left to turn it OFF
    {
      ledCtrl[0].offLeft = ledCtrl[0].offLeft - 1;
      ledCtrl[0].next = LEDCLR_OFF; // next command is to turn OFF
      osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT1, (uint32)ledCtrl[0].onTime*100);   /* Schedule event */
    }
  }
}

void ArcherLed2Update (void)
{
  if (ledCtrl[1].next == 0)
    HAL_TURN_OFF_LED2();
  else 
    HAL_TURN_ON_LED2();
  
  if (ledCtrl[1].next == 0) // Current command is to turn OFF
  {
    if (ledCtrl[1].onLeft != 0) // There is flashes left to turn it ON
    {
      ledCtrl[1].onLeft = ledCtrl[1].onLeft - 1;
      ledCtrl[1].next = ledCtrl[1].color; // next command is to turn ON
      osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT2, (uint32)ledCtrl[1].offTime*100);   /* Schedule event */
    }
  }
  else // Current command is to turn ON
  {
    if (ledCtrl[1].offLeft != 0) // There is flashes left to turn it OFF
    {
      ledCtrl[1].offLeft = ledCtrl[1].offLeft - 1;
      ledCtrl[1].next = LEDCLR_OFF; // next command is to turn OFF
      osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT2, (uint32)ledCtrl[1].onTime*100);   /* Schedule event */
    }
  }
}

void ArcherLed3Update (void)
{
  if (ledCtrl[2].next == 0)
    HAL_TURN_OFF_LED3();
  else 
    HAL_TURN_ON_LED3();
  
  if (ledCtrl[2].next == 0) // Current command is to turn OFF
  {
    if (ledCtrl[2].onLeft != 0) // There is flashes left to turn it ON
    {
      ledCtrl[2].onLeft = ledCtrl[2].onLeft - 1;
      ledCtrl[2].next = ledCtrl[2].color; // next command is to turn ON
      osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT3, (uint32)ledCtrl[2].offTime*100);   /* Schedule event */
    }
  }
  else // Current command is to turn ON
  {
    if (ledCtrl[2].offLeft != 0) // There is flashes left to turn it OFF
    {
      ledCtrl[2].offLeft = ledCtrl[2].offLeft - 1;
      ledCtrl[2].next = LEDCLR_OFF; // next command is to turn OFF
      osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT3, (uint32)ledCtrl[2].onTime*100);   /* Schedule event */
    }
  }
}
/***************************************************************************************************
 * @fn      HalLedSet
 *
 * @brief   Tun ON/OFF/TOGGLE given LEDs
 *
 * @param   led - bit mask value of leds to be turned ON/OFF/TOGGLE
 *          mode - BLINK, FLASH, TOGGLE, ON, OFF
 * @return  None
 ***************************************************************************************************/
uint8 HalLedSet (uint8 leds, uint8 mode)
{

#if (defined (BLINK_LEDS)) && (HAL_LED == TRUE)
  uint8 led;
  HalLedControl_t *sts;

  switch (mode)
  {
    case HAL_LED_MODE_BLINK:
      /* Default blink, 1 time, D% duty cycle */
      HalLedBlink (leds, 1, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME);
      break;

    case HAL_LED_MODE_FLASH:
      /* Default flash, N times, D% duty cycle */
      HalLedBlink (leds, HAL_LED_DEFAULT_FLASH_COUNT, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME);
      break;

    case HAL_LED_MODE_ON:
    case HAL_LED_MODE_OFF:
    case HAL_LED_MODE_TOGGLE:

      led = HAL_LED_1;
      leds &= HAL_LED_ALL;
      sts = HalLedStatusControl.HalLedControlTable;

      while (leds)
      {
        if (leds & led)
        {
          if (mode != HAL_LED_MODE_TOGGLE)
          {
            sts->mode = mode;  /* ON or OFF */
          }
          else
          {
            sts->mode ^= HAL_LED_MODE_ON;  /* Toggle */
          }
          HalLedOnOff (led, sts->mode);
          leds ^= led;
        }
        led <<= 1;
        sts++;
      }
      break;

    default:
      break;
  }

#elif (HAL_LED == TRUE)
  LedOnOff(leds, mode);
#else
  // HAL LED is disabled, suppress unused argument warnings
  (void) leds;
  (void) mode;
#endif /* BLINK_LEDS && HAL_LED   */

  return ( HalLedState );
}

/***************************************************************************************************
 * @fn      HalLedBlink
 *
 * @brief   Blink the leds
 *
 * @param   leds       - bit mask value of leds to be blinked
 *          numBlinks  - number of blinks
 *          percent    - the percentage in each period where the led
 *                       will be on
 *          period     - length of each cycle in milliseconds
 *
 * @return  None
 ***************************************************************************************************/
void HalLedBlink (uint8 leds, uint8 numBlinks, uint8 percent, uint16 period)
{
#if (defined (BLINK_LEDS)) && (HAL_LED == TRUE)
  uint8 led;
  HalLedControl_t *sts;

  if (leds && percent && period)
  {
    if (percent < 100)
    {
      led = HAL_LED_1;
      leds &= HAL_LED_ALL;
      sts = HalLedStatusControl.HalLedControlTable;

      while (leds)
      {
        if (leds & led)
        {
          /* Store the current state of the led before going to blinking if not already blinking */
          if(sts->mode < HAL_LED_MODE_BLINK )
          	preBlinkState |= (led & HalLedState);

          sts->mode  = HAL_LED_MODE_OFF;                    /* Stop previous blink */
          sts->time  = period;                              /* Time for one on/off cycle */
          sts->onPct = percent;                             /* % of cycle LED is on */
          sts->left  = numBlinks;                           /* Number of blink cycles */
          if (!numBlinks) sts->mode |= HAL_LED_MODE_FLASH;  /* Continuous */
          sts->next = osal_GetSystemClock();                /* Start now */
          sts->mode |= HAL_LED_MODE_BLINK;                  /* Enable blinking */
          leds ^= led;
        }
        led <<= 1;
        sts++;
      }
      // Cancel any overlapping timer for blink events
      osal_stop_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT);
      osal_set_event (Hal_TaskID, HAL_LED_BLINK_EVENT);
    }
    else
    {
      HalLedSet (leds, HAL_LED_MODE_ON);                    /* >= 100%, turn on */
    }
  }
  else
  {
    HalLedSet (leds, HAL_LED_MODE_OFF);                     /* No on time, turn off */
  }
#elif (HAL_LED == TRUE)
  percent = (leds & HalLedState) ? HAL_LED_MODE_OFF : HAL_LED_MODE_ON;
  HalLedOnOff (leds, percent);                              /* Toggle */
#else
  // HAL LED is disabled, suppress unused argument warnings
  (void) leds;
  (void) numBlinks;
  (void) percent;
  (void) period;
#endif /* BLINK_LEDS && HAL_LED */
}

#if (HAL_LED == TRUE)
/***************************************************************************************************
 * @fn      HalLedUpdate
 *
 * @brief   Update leds to work with blink
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void HalLedUpdate (void)
{
  uint8 led;
  uint8 pct;
  uint8 leds;
  HalLedControl_t *sts;
  uint32 time;
  uint16 next;
  uint16 wait;

  next = 0;
  led  = HAL_LED_1;
  leds = HAL_LED_ALL;
  sts = HalLedStatusControl.HalLedControlTable;

  /* Check if sleep is active or not */
  if (!HalLedStatusControl.sleepActive)
  {
    while (leds)
    {
      if (leds & led)
      {
        if (sts->mode & HAL_LED_MODE_BLINK)
        {
          time = osal_GetSystemClock();
          if (time >= sts->next)
          {
            if (sts->mode & HAL_LED_MODE_ON)
            {
              pct = 100 - sts->onPct;               /* Percentage of cycle for off */
              sts->mode &= ~HAL_LED_MODE_ON;        /* Say it's not on */
              HalLedOnOff (led, HAL_LED_MODE_OFF);  /* Turn it off */

              if ( !(sts->mode & HAL_LED_MODE_FLASH) )
              {
                sts->left--;                         // Not continuous, reduce count
              }
            }
            else if ( !(sts->left) && !(sts->mode & HAL_LED_MODE_FLASH) )
            {
              sts->mode ^= HAL_LED_MODE_BLINK;       // No more blinks
            }
            else
            {
              pct = sts->onPct;                      // Percentage of cycle for on
              sts->mode |= HAL_LED_MODE_ON;          // Say it's on
              HalLedOnOff( led, HAL_LED_MODE_ON );   // Turn it on
            }
            if (sts->mode & HAL_LED_MODE_BLINK)
            {
              wait = (((uint32)pct * (uint32)sts->time) / 100);
              sts->next = time + wait;
            }
            else
            {
              /* no more blink, no more wait */
              wait = 0;
              /* After blinking, set the LED back to the state before it blinks */
              HalLedSet (led, ((preBlinkState & led)!=0)?HAL_LED_MODE_ON:HAL_LED_MODE_OFF);
              /* Clear the saved bit */
              preBlinkState &= (led ^ 0xFF);
            }
          }
          else
          {
            wait = sts->next - time;  /* Time left */
          }

          if (!next || ( wait && (wait < next) ))
          {
            next = wait;
          }
        }
        leds ^= led;
      }
      led <<= 1;
      sts++;
    }

    if (next)
    {
      osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT, next);   /* Schedule event */
    }
  }
}

/***************************************************************************************************
 * @fn      HalLedOnOff
 *
 * @brief   Turns specified LED ON or OFF
 *
 * @param   leds - LED bit mask
 *          mode - LED_ON,LED_OFF,
 *
 * @return  none
 ***************************************************************************************************/
void HalLedOnOff (uint8 leds, uint8 mode)
{
  if (leds & HAL_LED_1)
  {
    if (mode == HAL_LED_MODE_ON)
    {
      HAL_TURN_ON_LED1();
    }
    else
    {
      HAL_TURN_OFF_LED1();
    }
  }

  if (leds & HAL_LED_2)
  {
    if (mode == HAL_LED_MODE_ON)
    {
      HAL_TURN_ON_LED2();
    }
    else
    {
      HAL_TURN_OFF_LED2();
    }
  }

  if (leds & HAL_LED_3)
  {
    if (mode == HAL_LED_MODE_ON)
    {
      HAL_TURN_ON_LED3();
    }
    else
    {
      HAL_TURN_OFF_LED3();
    }
  }

  if (leds & HAL_LED_4)
  {
    if (mode == HAL_LED_MODE_ON)
    {
      HAL_TURN_ON_LED4();
    }
    else
    {
      HAL_TURN_OFF_LED4();
    }
  }

  /* Remember current state */
  if (mode)
  {
    HalLedState |= leds;
  }
  else
  {
    HalLedState &= (leds ^ 0xFF);
  }
}
#endif /* HAL_LED */

/***************************************************************************************************
 * @fn      HalGetLedState
 *
 * @brief   Dim LED2 - Dim (set level) of LED2
 *
 * @param   none
 *
 * @return  led state
 ***************************************************************************************************/
uint8 HalLedGetState ()
{
#if (HAL_LED == TRUE)
  return HalLedState;
#else
  return 0;
#endif
}

/***************************************************************************************************
 * @fn      HalLedEnterSleep
 *
 * @brief   Store current LEDs state before sleep
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void HalLedEnterSleep( void )
{
#ifdef BLINK_LEDS
  /* Sleep ON */
  HalLedStatusControl.sleepActive = TRUE;
#endif /* BLINK_LEDS */

#if (HAL_LED == TRUE)
  /* Save the state of each led */
  HalSleepLedState = 0;
  HalSleepLedState |= HAL_STATE_LED1();
  HalSleepLedState |= HAL_STATE_LED2() << 1;
  HalSleepLedState |= HAL_STATE_LED3() << 2;
  HalSleepLedState |= HAL_STATE_LED4() << 3;

  /* TURN OFF all LEDs to save power */
  HalLedOnOff (HAL_LED_ALL, HAL_LED_MODE_OFF);
#endif /* HAL_LED */

}

/***************************************************************************************************
 * @fn      HalLedExitSleep
 *
 * @brief   Restore current LEDs state after sleep
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void HalLedExitSleep( void )
{
#if (HAL_LED == TRUE)
  /* Load back the saved state */
  HalLedOnOff(HalSleepLedState, HAL_LED_MODE_ON);

  /* Restart - This takes care BLINKING LEDS */
  HalLedUpdate();
#endif /* HAL_LED */

#ifdef BLINK_LEDS
  /* Sleep OFF */
  HalLedStatusControl.sleepActive = FALSE;
#endif /* BLINK_LEDS */
}

/***************************************************************************************************
 * @fn      ArcherLedInit
 *
 * @brief   Initial clock LED and color LED
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void ArcherLedInit()
{
  // Turn off LED power, P2_0 = 0
  P2_0 = 0;
  // Initialize Upper PCA9532
  HalI2CInit(0x60, I2CCLOCK);
  uint8 wdata0[9] = {0x12, 0x00, BRIGHTNESS_9532, 0x00, BRIGHTNESS_9532, 0x00, 0x00, 0x00, 0x00};
  HalI2CWrite(9, wdata0);
  // Initialize Lower PCA9532
  HalI2CInit(0x67, I2CCLOCK);
  HalI2CWrite(9, wdata0);
  // Initialize Color PCA9632
  HalI2CInit(0x62, I2CCLOCK);
  uint8 wdata1[10] = {0x80, 0x10, 0x01, BRIGHTNESS_9632_IND, BRIGHTNESS_9632_IND, BRIGHTNESS_9632_IND, 0x00, BRIGHTNESS_9632_GRP, 0x00, 0x00};
  HalI2CWrite(10, wdata1);
}

/***************************************************************************************************
 * @fn      ArcherHourLedSet
 *
 * @brief   Set Archer Hour LED
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void ArcherHourLedSet(uint8 hour, uint8 value)
{
  if (hour<12)
  {
    uint32 ledToSet = 0x01;
    ledToSet = ledToSet << archerHourTable[hour];
    if (value)
      archer9532 |= ledToSet;
    else
      archer9532 &= !ledToSet;
  }
}

/***************************************************************************************************
 * @fn      ArcherMinLedSet
 *
 * @brief   Set Archer Min LED
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void ArcherMinLedSet(uint8 min, uint8 value)
{
  if (min<16)
  {
    uint32 ledToSet = 0x01;
    ledToSet = ledToSet << archerMinTable[min];
    if (value)
      archer9532 |= ledToSet;
    else
      archer9532 &= !ledToSet;
  }
}

/***************************************************************************************************
 * @fn      ArcherClockLedUpdate
 *
 * @brief   Update Archer Clock LED
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void ArcherClockLedUpdate()
{
  // Upper LED
  if ((archer9532 & 0xffff0000) != (lastArcher9532 & 0xffff0000))
  {
    uint16 upperLED = archer9532 >> 16;
    uint8 wdata[5] = {0x16, 0x00, 0x00, 0x00, 0x00};
    for (uint8 wdata_B=1; wdata_B<5; wdata_B++)
    {
      for (uint8 wdata_led=0; wdata_led<4; wdata_led++)
      {
        wdata[wdata_B] = wdata[wdata_B] >> 2;
        if (upperLED & 0x0001)
          wdata[wdata_B] |= 0x80;
        upperLED = upperLED >> 1;
      }
    }
    HalI2CInit(0x60, I2CCLOCK);
    HalI2CWrite(5, wdata);
  }
  // Lower LED
  if ((archer9532 & 0x0000ffff) != (lastArcher9532 & 0x0000ffff))
  {
    uint16 lowerLED = archer9532 & 0xffff;
    uint8 wdata[5] = {0x16, 0x00, 0x00, 0x00, 0x00};
    for (uint8 wdata_B=1; wdata_B<5; wdata_B++)
    {
      for (uint8 wdata_led=0; wdata_led<4; wdata_led++)
      {
        wdata[wdata_B] = wdata[wdata_B] >> 2;
        if (lowerLED & 0x0001)
          wdata[wdata_B] |= 0x80;
        lowerLED = lowerLED >> 1;
      }
    }
    HalI2CInit(0x67, I2CCLOCK);
    HalI2CWrite(5, wdata);
  }
  lastArcher9532 = archer9532;
}

/***************************************************************************************************
***************************************************************************************************/