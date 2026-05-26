#ifndef BOARD_EPWM_H
#define BOARD_EPWM_H

#include "board/board_system.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Logical PWM channels for the SRCDAB bridge.
 * SRCDAB 桥路逻辑 PWM 通道。控制层只使用这些逻辑通道。
 */
typedef enum
{
    BOARD_EPWM_PRI_LEG_A = 0,    /* Primary bridge leg A / 原边桥臂 A */
    BOARD_EPWM_PRI_LEG_B,        /* Primary bridge leg B / 原边桥臂 B */
    BOARD_EPWM_SEC_LEG_A,        /* Secondary bridge leg A / 副边桥臂 A */
    BOARD_EPWM_SEC_LEG_B,        /* Secondary bridge leg B / 副边桥臂 B */
    BOARD_EPWM_CHANNEL_COUNT
} Board_EPWMChannel;

/* EPWM module mapping.
 * EPWM 模块映射。
 *
 * Change only these BASE macros when moving a logical channel to another EPWM.
 * GPIO PinMux for PWM9~PWM12 is pre-configured in board_gpio.c.
 *
 * 如果要把某个逻辑通道从 EPWM9 改到其它 EPWM，只改这里的 BASE 宏。
 * PWM9~PWM12 的 GPIO PinMux 已经在 board_gpio.c 中预先配置。
 */
#define BOARD_EPWM_PRI_LEG_A_BASE        EPWM9_BASE
#define BOARD_EPWM_PRI_LEG_B_BASE        EPWM10_BASE
#define BOARD_EPWM_SEC_LEG_A_BASE        EPWM11_BASE
#define BOARD_EPWM_SEC_LEG_B_BASE        EPWM12_BASE

#define BOARD_EPWM_PRI_LEG_A_ENABLE      1
#define BOARD_EPWM_PRI_LEG_B_ENABLE      1
#define BOARD_EPWM_SEC_LEG_A_ENABLE      1
#define BOARD_EPWM_SEC_LEG_B_ENABLE      1

/* Initialize EPWM hardware. Outputs remain tripped low after init.
 * 初始化 EPWM 硬件。初始化完成后输出仍通过 TripZone 保持低电平。
 */
void Board_initEPWM(void);

/* Output safety control.
 * 输出安全控制。
 */
void Board_EPWM_forceLowAll(void);       /* Force OST trip low / 强制 OST 关断为低 */
void Board_EPWM_enableOutputs(void);     /* Clear OST if no fault / 无故障时清除 OST */
void Board_EPWM_disableOutputs(void);    /* Re-force OST trip / 重新强制 OST */

/* EPWM9 ADC trigger control.
 * EPWM9 ADC 触发控制。
 *
 * These functions configure the EPWM9 ADCSOCA event used by the control ADC
 * path. ADC SOC channel setup and result reads stay in board_adc.c.
 *
 * 这些函数配置闭环 ADC 使用的 EPWM9 ADCSOCA 事件。ADC SOC 通道配置和结果
 * 读取放在 board_adc.c 中。
 */
void Board_EPWM_initADCTrigger(void);
void Board_EPWM_enableADCTrigger(void);
void Board_EPWM_disableADCTrigger(void);

/* Frequency and deadband update.
 * 频率和死区更新。
 */
void Board_EPWM_setFrequencyHz(uint32_t freq_hz);
void Board_EPWM_setFrequencyHzSafe(uint32_t freq_hz);
void Board_EPWM_setDeadBandNs(uint16_t deadband_ns);

/* Duty update. duty is clamped to 0.0f~1.0f.
 * 占空比更新。duty 会被限制在 0.0f~1.0f。
 */
void Board_EPWM_setDuty(Board_EPWMChannel ch, float duty);
void Board_EPWM_setDutyAll(float duty);

/* Phase update.
 * 相位更新。
 *
 * User inputs are absolute phase values. Internally all phases are rebased
 * relative to BOARD_EPWM_PRI_LEG_A, which maps to EPWM9 and remains the
 * hardware sync master.
 *
 * 用户输入的是绝对相位。内部会全部换算为相对 BOARD_EPWM_PRI_LEG_A 的相位；
 * BOARD_EPWM_PRI_LEG_A 映射到 EPWM9，仍是硬件同步主模块。
 */
void Board_EPWM_setPhaseDeg(Board_EPWMChannel ch, float phase_deg);
void Board_EPWM_setPhaseRad(Board_EPWMChannel ch, float phase_rad);
void Board_EPWM_setPhaseCount(Board_EPWMChannel ch, uint16_t phase_count);

void Board_EPWM_setAllPhaseDeg(float priA_deg,
                               float priB_deg,
                               float secA_deg,
                               float secB_deg);

uint16_t Board_EPWM_getTBPRD(void);
uint32_t Board_EPWM_getTBCLKHz(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_EPWM_H */
