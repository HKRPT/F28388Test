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
 * 初始化板级总控默认值。
 */
void Board_SystemInit(void)
{
    gBoardSystem.state = BOARD_STATE_POWER_ON;
    gBoardSystem.pwm_freq_hz = BOARD_PWM_DEFAULT_FREQ_HZ;
    gBoardSystem.pwm_deadband_ns = BOARD_PWM_DEFAULT_DEADBAND_NS;
    gBoardSystem.pwm_output_enabled = false;
    gBoardSystem.fault_latched = false;
}

/* Board-level PWM safety task.
 * 板级 PWM 安全总控任务。
 *
 * This is not the SRCDAB power-stage state machine. It only decides whether
 * PWM outputs are forced low or released.
 * 这不是 SRCDAB 功率级业务状态机，只负责 PWM 输出强制低电平或解除强制。
 */
void Board_SystemTask(void)
{
    switch (gBoardSystem.state)
    {
        case BOARD_STATE_POWER_ON:
            /* Power-on safety: keep all PWM pins low.
             * 上电安全：所有 PWM 引脚保持低电平。
             */
            Board_EPWM_forceLowAll();
            gBoardSystem.state = BOARD_STATE_INIT;
            break;

        case BOARD_STATE_INIT:
            /* Board init guard state: still keep PWM low.
             * 板级初始化保护态：继续保持 PWM 低电平。
             */
            Board_EPWM_forceLowAll();
            gBoardSystem.state = BOARD_STATE_PWM_SAFE_OFF;
            break;

        case BOARD_STATE_PWM_SAFE_OFF:
            /* Safe-off state: wait for enable request and no latched fault.
             * 安全关闭态：等待输出使能请求，并确认没有故障锁存。
             */
            Board_EPWM_forceLowAll();
            if ((!gBoardSystem.fault_latched) && gBoardSystem.pwm_output_enabled)
            {
                gBoardSystem.state = BOARD_STATE_PWM_READY;
            }
            break;

        case BOARD_STATE_PWM_READY:
            /* Release PWM outputs only from this explicit ready state.
             * 只在明确进入 READY 后解除 PWM 软件强制。
             */
            Board_EPWM_enableOutputs();
            gBoardSystem.state = BOARD_STATE_RUN;
            break;

        case BOARD_STATE_RUN:
            /* Runtime guard: fault or disable request returns PWM to safe-off.
             * 运行保护：故障或关闭请求都会让 PWM 回到安全关闭。
             */
            if (gBoardSystem.fault_latched)
            {
                gBoardSystem.state = BOARD_STATE_FAULT;
            }
            else if (!gBoardSystem.pwm_output_enabled)
            {
                Board_EPWM_disableOutputs();
                gBoardSystem.state = BOARD_STATE_PWM_SAFE_OFF;
            }
            break;

        case BOARD_STATE_FAULT:
        default:
            /* Fault state is latched until Board_SystemClearFault() is called.
             * 故障态保持锁存，直到调用 Board_SystemClearFault()。
             */
            Board_EPWM_forceLowAll();
            gBoardSystem.pwm_output_enabled = false;
            gBoardSystem.state = BOARD_STATE_FAULT;
            break;
    }
}

/* Latch a board-level fault and force the supervisor to FAULT.
 * 锁存板级故障，并强制总控进入 FAULT。
 */
void Board_SystemSetFault(void)
{
    gBoardSystem.fault_latched = true;
    gBoardSystem.state = BOARD_STATE_FAULT;
}

/* Clear latched fault and return to safe-off.
 * 清除故障锁存，并回到 PWM 安全关闭态。
 */
void Board_SystemClearFault(void)
{
    gBoardSystem.fault_latched = false;
    if (gBoardSystem.state == BOARD_STATE_FAULT)
    {
        gBoardSystem.state = BOARD_STATE_PWM_SAFE_OFF;
    }
}

/* Return PWM output enable request.
 * 返回 PWM 输出使能请求状态。
 */
bool Board_SystemIsPWMEnabled(void)
{
    return gBoardSystem.pwm_output_enabled;
}

/* Return latched fault state.
 * 返回故障锁存状态。
 */
bool Board_SystemIsFault(void)
{
    return gBoardSystem.fault_latched;
}
