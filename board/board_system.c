#include "board/board.h"

Board_SystemHandle gBoardSystem;

void Board_init(void)
{
    Board_SystemInit();
    Board_initGPIO();
    Board_initEPWM();
    Board_initADC();
    Board_initTripZone();
    Board_initInterrupts();
}

/* Initialize board supervisor defaults.
 * 初始化板级总控默认值。 */
void Board_SystemInit(void)
{
    gBoardSystem.pwm_freq_hz = BOARD_PWM_DEFAULT_FREQ_HZ;
    gBoardSystem.pwm_deadband_ns = BOARD_PWM_DEFAULT_DEADBAND_NS;
    gBoardSystem.pwm_output_enabled = false;
    gBoardSystem.fault_latched = false;
}

/* Board-level PWM safety guard.
 * 板级 PWM 安全守护任务。
 *
 * DAB business state is owned by ControlLoop. This task only keeps PWM forced
 * low when output is not requested or a fault is latched.
 *
 * DAB 业务状态由 ControlLoop 管理。这里只在未请求输出或故障锁存时保持 PWM 强制低。
 */
void Board_SystemTask(void)
{
    if (gBoardSystem.fault_latched)
    {
        gBoardSystem.pwm_output_enabled = false;
        Board_EPWM_forceLowAll();
    }
    else if (gBoardSystem.pwm_output_enabled)
    {
        Board_EPWM_enableOutputs();
    }
    else
    {
        Board_EPWM_forceLowAll();
    }
}

/* Latch a board-level fault.
 * 锁存板级故障。 */
void Board_SystemSetFault(void)
{
    gBoardSystem.fault_latched = true;
}

/* Clear latched fault. PWM output remains disabled until requested again.
 * 清除故障锁存。PWM 输出保持关闭，直到重新请求使能。 */
void Board_SystemClearFault(void)
{
    gBoardSystem.fault_latched = false;
    gBoardSystem.pwm_output_enabled = false;
}

/* Return PWM output enable request.
 * 返回 PWM 输出使能请求状态。 */
bool Board_SystemIsPWMEnabled(void)
{
    return gBoardSystem.pwm_output_enabled;
}

/* Return latched fault state.
 * 返回故障锁存状态。 */
bool Board_SystemIsFault(void)
{
    return gBoardSystem.fault_latched;
}
