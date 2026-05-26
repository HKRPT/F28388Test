#ifndef BOARD_H
#define BOARD_H

#include "driverlib.h"
#include "device.h"
#include "board/board_system.h"
#include "board/board_gpio.h"
#include "board/board_epwm.h"
#include "board/board_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 * Exported Functions
 *------------------------------------------------------------------------------*/
void Board_init(void);
void Board_initTripZone(void);
void Board_initInterrupts(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
