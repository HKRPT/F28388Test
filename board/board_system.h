#ifndef BOARD_SYSTEM_H
#define BOARD_SYSTEM_H

#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PrimaryTurn 20
#define SecondTurn  5
#define TurnsProportion 4.0f

/*=============================================================================
 * Board clock tree / 板级时钟链路
 *
 * Keep DriverLib enum values separate from numeric divider values.
 * DriverLib 枚举只传给 TI API；数值计算必须使用 *_NUM 宏，避免把枚举当分频值。
 *=============================================================================*/
#define BOARD_SYSCLK_HZ                  DEVICE_SYSCLK_FREQ

#define BOARD_EPWMCLK_DIV_NUM            2UL
#define BOARD_EPWMCLK_HZ                 (BOARD_SYSCLK_HZ / BOARD_EPWMCLK_DIV_NUM)

#define BOARD_EPWM_CLKDIV_ENUM           EPWM_CLOCK_DIVIDER_1
#define BOARD_EPWM_HSCLKDIV_ENUM         EPWM_HSCLOCK_DIVIDER_1

#define BOARD_EPWM_CLKDIV_NUM            1UL
#define BOARD_EPWM_HSCLKDIV_NUM          1UL

#define BOARD_EPWM_TBCLK_HZ              (BOARD_EPWMCLK_HZ / \
                                          BOARD_EPWM_CLKDIV_NUM / \
                                          BOARD_EPWM_HSCLKDIV_NUM)

/* Default PWM parameters.
 * PWM 默认参数。
 */
#define BOARD_PWM_DEFAULT_FREQ_HZ        100000UL   /* Default frequency / 默认频率 */
#define BOARD_PWM_DEFAULT_DEADBAND_NS    200U       /* Default deadband / 默认死区 */

/* Board-level PWM supervisor state.
 * 板级 PWM 总控状态。这里只管理 PWM 安全开关，不包含 SRCDAB 业务状态机。
 */
typedef enum
{
    BOARD_STATE_POWER_ON = 0,    /* 上电初始态 / Power-on entry state */
    BOARD_STATE_INIT,            /* 板级初始化态 / Board initialization */
    BOARD_STATE_PWM_SAFE_OFF,    /* PWM 安全关闭 / PWM forced low */
    BOARD_STATE_PWM_READY,       /* PWM 准备使能 / Ready to release outputs */
    BOARD_STATE_RUN,             /* PWM 输出运行 / PWM outputs released */
    BOARD_STATE_FAULT            /* 故障锁存 / Fault latched */
} Board_State;

/* Global board runtime handle.
 * 板级全局运行句柄：保存 PWM 默认参数、输出使能请求和故障锁存。
 */
typedef struct
{
    Board_State state;           /* 当前板级状态 / Current board state */

    uint32_t pwm_freq_hz;        /* 当前 PWM 频率 / Current PWM frequency */
    uint16_t pwm_deadband_ns;    /* 当前死区时间 / Current deadband time */

    bool pwm_output_enabled;     /* PWM 输出使能请求 / PWM output enable request */
    bool fault_latched;          /* 故障锁存标志 / Latched fault flag */
} Board_SystemHandle;

extern Board_SystemHandle gBoardSystem;

/* Initialize board-level runtime state.
 * 初始化板级运行状态。
 */
void Board_SystemInit(void);

/* Run the board-level PWM safety supervisor.
 * 运行板级 PWM 安全总控任务。
 */
void Board_SystemTask(void);

/* Latch and clear board fault.
 * 设置和清除板级故障锁存。
 */
void Board_SystemSetFault(void);
void Board_SystemClearFault(void);

/* Query output-enable request and fault state.
 * 查询 PWM 输出使能请求和故障状态。
 */
bool Board_SystemIsPWMEnabled(void);
bool Board_SystemIsFault(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_SYSTEM_H */
