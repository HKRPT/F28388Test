#ifndef BOARD_SCI_H
#define BOARD_SCI_H

#include "driverlib.h"
#include "device.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 * SCIB debug port hardware mapping.
 * SCIB 调试串口硬件映射。
 *
 * CH340 wiring / CH340 接线：
 *   DSP GPIO9  / SCIB_TX -> CH340 RXD
 *   DSP GPIO11 / SCIB_RX <- CH340 TXD
 *------------------------------------------------------------------------------*/
#define BOARD_SCIB_BASE             SCIB_BASE
#define BOARD_SCIB_DEFAULT_BAUD     921600UL

#define BOARD_SCIB_TX_GPIO          9U
#define BOARD_SCIB_TX_PIN_CONFIG    GPIO_9_SCIB_TX

#define BOARD_SCIB_RX_GPIO          11U
#define BOARD_SCIB_RX_PIN_CONFIG    GPIO_11_SCIB_RX

/* Initialize SCIB GPIO pinmux for the CH340 debug UART.
 * 初始化 CH340 调试串口使用的 SCIB GPIO 复用。
 *
 * Input:
 *   None.
 * Output:
 *   None.
 */
void Board_SCIB_initGPIO(void);

/* Initialize SCIB as an 8N1 FIFO UART.
 * 将 SCIB 初始化为 8N1 FIFO 串口。
 *
 * Input:
 *   baudRate - UART baud rate, such as BOARD_SCIB_DEFAULT_BAUD.
 *              串口波特率，例如 BOARD_SCIB_DEFAULT_BAUD。
 * Output:
 *   None.
 */
void Board_SCIB_init(uint32_t baudRate);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_SCI_H */
