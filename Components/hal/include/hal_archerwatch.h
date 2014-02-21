#ifndef HAL_CLOCK_H
#define HAL_CLOCK_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_board.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*
 * Initialize Clock Service.
 */
extern void ArcherClockInit( void );
/*
 * Archer Clock Set
 *
 */
extern void ArcherClockSet (uint8 hour, uint8 minute, uint8 second);
/*
 * Archer Clock Update
 *
 */
extern void ArcherClockUpdate (void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
