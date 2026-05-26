#ifndef ISR_H
#define ISR_H

#include "driverlib.h"
#include "device.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ADCC1 interrupt service routine.
 * ADCC1 中断服务函数。
 *
 * Trigger path:
 * EPWM9 SOCA -> ADCC/ADCD SOC0~SOC2 -> ADCC SOC2 EOC -> ADCC1 ISR.
 *
 * 触发链路：
 * EPWM9 SOCA -> ADCC/ADCD SOC0~SOC2 -> ADCC SOC2 EOC -> ADCC1 ISR。
 */
__interrupt void adcc1ISR(void);

#ifdef __cplusplus
}
#endif

#endif /* ISR_H */
