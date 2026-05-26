#include "driverlib.h"
#include "device.h"
#include "board/board.h"
#include "control/ControlLoop.h"

/* Application entry.
 * 应用入口。
 *
 * main only performs device and board initialization. It does not contain EPWM
 * register details or SRCDAB control logic.
 * main 只负责设备和板级初始化，不写 EPWM 寄存器细节，也不写 SRCDAB 控制算法。
 */
void main(void)
{
    /* TI device-level initialization.
     * TI 器件级初始化。
     */
    Device_init();
    Device_initGPIO();

    /* Interrupt controller initialization.
     * 中断控制器初始化。
     */
    Interrupt_initModule();
    Interrupt_initVectorTable();

    /* Board hardware modules. EPWM init ends in force-low state.
     * 板级运行参数默认值。
     */
    Board_init();

    /* Control-loop placeholder initialization.
     * 板级硬件模块初始化。EPWM 初始化结束后保持强制低电平。
     */
    ControlLoop_init();

    /* EPWM9 hardware ADC trigger path.
     * EPWM9 硬件 ADC 触发链路。
     *
     * PWM outputs may still be held low by TripZone, but EPWM9 time-base keeps
     * running after TBCLKSYNC is enabled, so SOCA can trigger ADC for bring-up
     * and debug.
     *
     * PWM 输出可以继续被 TripZone 锁低；但 TBCLKSYNC 打开后 EPWM9 时基仍会运行，
     * 因此 SOCA 可以先用于 ADC 调试采样。
     */
    Board_EPWM_initADCTrigger();
    Board_EPWM_enableADCTrigger();

    /* Enable CPU global interrupt and realtime debug interrupt.
     * 使能 CPU 全局中断和实时调试中断。
     */
    EINT;
    ERTM;

    while (1)
    {
        /* Board-level PWM safety supervisor and non-realtime control task.
         * 这里只运行板级 PWM 安全总控，不运行 SRCDAB 闭环控制。
         */
        Board_SystemTask();
        ControlLoop_slowTask();
    }
}
