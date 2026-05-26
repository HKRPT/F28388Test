#include "board.h"
#include "isr/isr.h"

/* Board interrupt registration.
 * 板级中断注册。
 *
 * Keep existing interrupt setup here. This file owns interrupt registration;
 * do not create another board interrupt source file with a duplicate
 * Board_initInterrupts().
 *
 * 中断注册统一放在这里，不新建第二个中断初始化文件，也不重复定义
 * Board_initInterrupts()。
 */
void Board_initInterrupts(void)
{
    /* ADCC1 is the ADC sampling-complete ISR entry.
     * ADCC1 是当前 ADC 采样完成中断入口。
     *
     * No EPWM ISR is registered here. ADC SOC is triggered by the EPWM9 SOCA
     * hardware event once per PWM period at the EPWM9A rising edge.
     *
     * 这里不注册 EPWM ISR。ADC SOC 由 EPWM9 SOCA 硬件事件在 EPWM9A 上升沿、
     * 每个 PWM 周期触发一次。
     */
    Interrupt_register(INT_ADCC1, &adcc1ISR);
    Interrupt_enable(INT_ADCC1);
}
