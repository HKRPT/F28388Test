#include "isr.h"
#include "board/board_adc.h"
#include "control/ControlLoop.h"

#define CONTROL_ISR_PROFILING_ENABLE    0U
#define CONTROL_ISR_PROFILING_GPIO      31U

#if CONTROL_ISR_PROFILING_ENABLE
#define CONTROL_ISR_PROFILE_TOGGLE()    GPIO_togglePin(CONTROL_ISR_PROFILING_GPIO)
#else
#define CONTROL_ISR_PROFILE_TOGGLE()    do { } while (0)
#endif

/* ADCC1 ISR.
 * ADCC1 中断。
 *
 * EPWM9 SOCA triggers all control ADC SOCs once per PWM period at the EPWM9A
 * rising edge. ADCC1 is sourced from ADCC SOC2 EOC, the last control SOC.
 * Control code may update
 * ePWM phase shadow/TBPHS values here; slave ePWM modules load phase on the
 * next EPWM9 sync at CTR=ZERO, not in the middle of the active PWM cycle.
 *
 * EPWM9 SOCA 在 EPWM9A 上升沿、每个 PWM 周期触发一次全部控制 ADC SOC。
 * ADCC1 来自最后一个控制 SOC2 的 EOC。控制代码可在这里更新 ePWM phase
 * shadow/TBPHS；从 ePWM 在下一次 EPWM9 CTR=ZERO 同步点加载相位，不在当前
 * PWM 周期中途改变边沿。
 */
__interrupt void adcc1ISR(void)
{
    bool adcdComplete;

    CONTROL_ISR_PROFILE_TOGGLE();

    /* ADCC and ADCD are both triggered by the same EPWM9 SOCA event.
     * ADCC1 ISR is sourced from ADCC SOC2 EOC, so ADCC SOC0~SOC2 are complete.
     * ADCD ADCINT1 is sourced from ADCD SOC2 EOC and is checked before reading
     * ADCDRESULT0~2.
     *
     * ADCC 和 ADCD 都由同一个 EPWM9 SOCA 触发。ADCC1 ISR 来自 ADCC
     * SOC2 EOC，因此 ADCC SOC0~SOC2 已完成。读取 ADCDRESULT0~2 前，先检查
     * ADCD SOC2 EOC 对应的 ADCINT1 标志。
     */
    adcdComplete = ADC_getInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);

    if (adcdComplete)
    {
        gAdcDNotReady = false;
        /*进行实际值转换*/
        Board_ADC_UpdateCacheFromResult();

        ControlLoop_adcISR();

        ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
    }
    else
    {
        gAdcDNotReady = true;
    }

    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

    if (ADC_getInterruptOverflowStatus(ADCC_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCC_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
    }

    if (ADC_getInterruptOverflowStatus(ADCD_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCD_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
    }

    CONTROL_ISR_PROFILE_TOGGLE();

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
