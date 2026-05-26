#ifndef BOARD_GPIO_H
#define BOARD_GPIO_H

#include "driverlib.h"
#include "device.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize board GPIO pinmux.
 * 初始化板级 GPIO PinMux。
 *
 * Current implementation pre-configures EPWM9~EPWM12 A/B pins only.
 * 当前实现只预配置 EPWM9~EPWM12 的 A/B 引脚。
 */
void Board_initGPIO(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_GPIO_H */
