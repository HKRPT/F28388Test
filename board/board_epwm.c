#include "board/board_system.h"
#include "board/board_epwm.h"

/* Local guard limits.
 * 本模块内部保护限制：避免频率非法、TBPRD 溢出、死区过大。
 */
#define BOARD_EPWM_MIN_FREQ_HZ       2000UL
#define BOARD_EPWM_MAX_FREQ_HZ       200000UL
#define BOARD_EPWM_MAX_TBPRD         0x7FFFU
#define BOARD_EPWM_MAX_DEADBAND_NS   5000U
#define BOARD_PI_F                   3.14159265358979323846f
#define PHASE_COUNT_MIN              0U
#define PHASE_COUNT_MAX(tbprd)       ((uint16_t)(((tbprd) > 0U) ? \
                                      (2U * (tbprd) - 1U) : 0U))

/* AQ reference waveform for EPWMxA. EPWMxB is produced by DeadBand.
 * EPWMxA 的 AQ 参考波形。EPWMxB 由 DeadBand 派生。
 */
#define BOARD_EPWM_AQ_ACTION_A   (EPWM_AQ_OUTPUT_HIGH_ZERO        | \
                                  EPWM_AQ_OUTPUT_LOW_UP_CMPA      | \
                                  EPWM_AQ_OUTPUT_HIGH_DOWN_CMPA)

/* Runtime data for each logical EPWM channel.
 * 每个逻辑 EPWM 通道的运行时数据。
 */
typedef struct
{
    uint32_t base;          /* EPWM register base / EPWM 寄存器基地址 */
    bool enabled;           /* Channel enable flag / 通道使能标志 */
    float duty;             /* Duty ratio 0.0f~1.0f / 占空比 */
    float phase_deg_abs;    /* User absolute phase / 用户输入的绝对相位 */
    uint16_t phase_count;   /* TBPHS value / TBPHS 相位计数 */
    EPWM_SyncCountMode count_mode_after_sync; /* UP/DOWN after sync / 同步后计数方向 */
} Board_EPWMRuntime;

/* Default phase plan.
 * 默认相位：原边 A、副边 A 为 0 度；原边 B、副边 B 为 180 度。
 */
static Board_EPWMRuntime gEpwm[BOARD_EPWM_CHANNEL_COUNT] =
{
    {BOARD_EPWM_PRI_LEG_A_BASE, (bool)BOARD_EPWM_PRI_LEG_A_ENABLE, 0.0f, 0.0f,   0U, EPWM_COUNT_MODE_UP_AFTER_SYNC},
    {BOARD_EPWM_PRI_LEG_B_BASE, (bool)BOARD_EPWM_PRI_LEG_B_ENABLE, 0.0f, 180.0f, 0U, EPWM_COUNT_MODE_UP_AFTER_SYNC},
    {BOARD_EPWM_SEC_LEG_A_BASE, (bool)BOARD_EPWM_SEC_LEG_A_ENABLE, 0.0f, 0.0f,   0U, EPWM_COUNT_MODE_UP_AFTER_SYNC},
    {BOARD_EPWM_SEC_LEG_B_BASE, (bool)BOARD_EPWM_SEC_LEG_B_ENABLE, 0.0f, 180.0f, 0U, EPWM_COUNT_MODE_UP_AFTER_SYNC},
};

static uint16_t gEpwmTbprd;
static uint16_t gEpwmDeadbandCount;

/* Validate channel index.
 * 检查通道索引是否合法。
 */
static bool Board_EPWM_isValidChannel(Board_EPWMChannel ch)
{
    return ((uint16_t)ch < (uint16_t)BOARD_EPWM_CHANNEL_COUNT);
}

/* Get EPWM base from logical channel.
 * 根据逻辑通道获取 EPWM 基地址。
 */
static uint32_t Board_EPWM_getBase(Board_EPWMChannel ch)
{
    return gEpwm[(uint16_t)ch].base;
}

/* Check whether a logical channel is enabled.
 * 检查逻辑通道是否启用。
 */
static bool Board_EPWM_isChannelEnabled(Board_EPWMChannel ch)
{
    return (Board_EPWM_isValidChannel(ch) && gEpwm[(uint16_t)ch].enabled);
}

/* Clamp duty command into 0.0f~1.0f.
 * 将占空比限制在 0.0f~0.95f。
 */
static float Board_EPWM_clampDuty(float duty)
{
    if (duty < 0.0f)
    {
        return 0.0f;
    }
    if (duty > 0.95f)
    {
        return 0.95f;
    }
    return duty;
}

/* Normalize phase into [0, 360) degrees.
 * 将相位归一化到 [0, 360) 度。
 */
static float Board_EPWM_normalizePhaseDeg(float phase_deg)
{
    while (phase_deg < 0.0f)
    {
        phase_deg += 360.0f;
    }
    while (phase_deg >= 360.0f)
    {
        phase_deg -= 360.0f;
    }
    return phase_deg;
}

/* Calculate TBPRD for up-down counting.
 * 计算上下计数模式的 TBPRD：PWM frequency = TBCLK / (2 * TBPRD)。
 */
static uint16_t Board_EPWM_calcTBPRD(uint32_t freq_hz)
{
    uint32_t tbprd;

    if (freq_hz < BOARD_EPWM_MIN_FREQ_HZ)
    {
        freq_hz = BOARD_EPWM_MIN_FREQ_HZ;
    }
    else if (freq_hz > BOARD_EPWM_MAX_FREQ_HZ)
    {
        freq_hz = BOARD_EPWM_MAX_FREQ_HZ;
    }

    tbprd = BOARD_EPWM_TBCLK_HZ / (2UL * freq_hz);
    if (tbprd == 0UL)
    {
        tbprd = 1UL;
    }
    else if (tbprd > BOARD_EPWM_MAX_TBPRD)
    {
        tbprd = BOARD_EPWM_MAX_TBPRD;
    }

    return (uint16_t)tbprd;
}

/* Convert duty ratio to CMPA.
 * 将占空比换算成 CMPA。duty=0.5f 时 CMPA=TBPRD/2。
 */
static uint16_t Board_EPWM_calcCmpA(float duty)
{
    uint32_t cmpa;

    duty = Board_EPWM_clampDuty(duty);
    cmpa = (uint32_t)((float)gEpwmTbprd * duty + 0.5f);
    if (cmpa > gEpwmTbprd)
    {
        cmpa = gEpwmTbprd;
    }

    return (uint16_t)cmpa;
}

/* Convert deadband nanoseconds to TBCLK counts with 64-bit math.
 * 使用 64 位计算把死区 ns 换算成 TBCLK 计数，避免非整数 MHz TBCLK 精度损失。
 */
static uint16_t Board_EPWM_calcDeadbandCount(uint16_t deadband_ns)
{
    uint64_t count;

    if (deadband_ns > BOARD_EPWM_MAX_DEADBAND_NS)
    {
        deadband_ns = BOARD_EPWM_MAX_DEADBAND_NS;
    }

    count = ((uint64_t)BOARD_EPWM_TBCLK_HZ * (uint64_t)deadband_ns +
             999999999ULL) / 1000000000ULL;
    if (count > BOARD_EPWM_MAX_TBPRD)
    {
        count = BOARD_EPWM_MAX_TBPRD;
    }

    return (uint16_t)count;
}

/* Calculate relative phase against PRI_LEG_A and map it into TBPHS.
 * 以 PRI_LEG_A 为基准计算相对相位，并映射到 TBPHS。
 *
 * Up-down counting note:
 * One full PWM cycle is 2 * TBPRD counts. TBPHS should stay within 0~TBPRD.
 * For phase > 180 deg, use (2*TBPRD - count) and DOWN_AFTER_SYNC.
 *
 * 上下计数模式说明：
 * 一个完整 PWM 周期是 2 * TBPRD 个计数。TBPHS 不应直接写大于 TBPRD 的值。
 * 对于大于 180 度的相位，使用 (2*TBPRD - count)，并设置同步后向下计数。
 */
static void Board_EPWM_calcRelativePhase(Board_EPWMChannel ch)
{
    uint16_t index = (uint16_t)ch;
    float master_phase;
    float relative_deg;
    float raw_count_f;
    uint32_t raw_count;

    master_phase = Board_EPWM_normalizePhaseDeg(
        gEpwm[(uint16_t)BOARD_EPWM_PRI_LEG_A].phase_deg_abs);
    relative_deg = Board_EPWM_normalizePhaseDeg(gEpwm[index].phase_deg_abs -
                                                master_phase);

    raw_count_f = (relative_deg / 360.0f) * (2.0f * (float)gEpwmTbprd);
    raw_count = (uint32_t)(raw_count_f + 0.5f);
    raw_count %= (uint32_t)(2UL * (uint32_t)gEpwmTbprd);

    if (raw_count <= (uint32_t)gEpwmTbprd)
    {
        gEpwm[index].phase_count = (uint16_t)raw_count;
        gEpwm[index].count_mode_after_sync = EPWM_COUNT_MODE_UP_AFTER_SYNC;
    }
    else
    {
        gEpwm[index].phase_count = (uint16_t)(2UL * (uint32_t)gEpwmTbprd -
                                             raw_count);
        gEpwm[index].count_mode_after_sync = EPWM_COUNT_MODE_DOWN_AFTER_SYNC;
    }
}

/* Configure TripZone actions so either TZA or TZB forces final pins low.
 * 配置 TripZone 动作：TZA/TZB 都强制最终 EPWM 引脚为低电平。
 *
 * TripZone is safer than AQ software force when complementary DeadBand is used,
 * because TripZone acts later in the EPWM output path.
 *
 * 在互补 DeadBand 场景下，TripZone 比 AQ 软件强制更安全，因为 TripZone
 * 位于 EPWM 输出链路后级。
 */
static void Board_EPWM_applyTripZoneSafeState(uint32_t base)
{
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
}

/* Force one-shot TripZone.
 * 触发 one-shot TripZone，锁住低电平输出。
 */
static void Board_EPWM_forceTrip(uint32_t base)
{
    EPWM_forceTripZoneEvent(base, EPWM_TZ_FORCE_EVENT_OST);
}

/* Clear one-shot TripZone.
 * 清除 one-shot TripZone。调用前必须确认没有 fault_latched。
 */
static void Board_EPWM_clearTrip(uint32_t base)
{
    EPWM_clearTripZoneFlag(base, EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);
}

/* Apply current TBPRD to one channel shadow register.
 * 将当前 TBPRD 写入某一路周期 shadow 寄存器。
 */
static void Board_EPWM_applyPeriod(Board_EPWMChannel ch)
{
    EPWM_setTimeBasePeriod(Board_EPWM_getBase(ch), gEpwmTbprd);
}

/* Apply current duty to one channel.
 * 将当前占空比写入某一路 CMPA shadow 寄存器。
 */
static void Board_EPWM_applyDuty(Board_EPWMChannel ch)
{
    uint32_t base = Board_EPWM_getBase(ch);
    uint16_t cmpa = Board_EPWM_calcCmpA(gEpwm[(uint16_t)ch].duty);

    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, cmpa);
}

/* Apply current phase mapping to one channel.
 * 将当前相位映射写入某一路 TBPHS 和同步后计数方向。
 */
static void Board_EPWM_applyPhase(Board_EPWMChannel ch)
{
    uint32_t base = Board_EPWM_getBase(ch);
    uint16_t index = (uint16_t)ch;

    Board_EPWM_calcRelativePhase(ch);
    EPWM_setPhaseShift(base, gEpwm[index].phase_count);
    EPWM_setCountModeAfterSync(base, gEpwm[index].count_mode_after_sync);
}

/* Recalculate and apply phase for all enabled channels.
 * 重新计算并应用所有已启用通道的相位。
 */
static void Board_EPWM_applyAllPhase(void)
{
    uint16_t i;

    for (i = 0U; i < (uint16_t)BOARD_EPWM_CHANNEL_COUNT; i++)
    {
        if (Board_EPWM_isChannelEnabled((Board_EPWMChannel)i))
        {
            Board_EPWM_applyPhase((Board_EPWMChannel)i);
        }
    }
}

/* Apply current RED/FED deadband counts.
 * 写入当前 RED/FED 死区计数。
 */
static void Board_EPWM_applyDeadband(Board_EPWMChannel ch)
{
    uint32_t base = Board_EPWM_getBase(ch);

    EPWM_setRisingEdgeDelayCount(base, gEpwmDeadbandCount);
    EPWM_setFallingEdgeDelayCount(base, gEpwmDeadbandCount);
}

/* Configure Active High Complementary DeadBand with shadow load on TBCTR=ZERO.
 * 配置 Active High Complementary 互补死区，并设置 shadow 在 TBCTR=ZERO 加载。
 */
static void Board_EPWM_configDeadband(uint32_t base)
{
    EPWM_setDeadBandControlShadowLoadMode(base, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCountShadowLoadMode(base, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCountShadowLoadMode(base, EPWM_FED_LOAD_ON_CNTR_ZERO);

    EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);

    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);

    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_A, false);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_B, false);
}

/* Configure one EPWM module.
 * 配置单个 EPWM 模块。
 *
 * PRI_LEG_A maps to EPWM9 and is the sync master. Other channels sync to
 * EPWM9 sync-out and use
 * TBPHS plus count direction to express their relative phase.
 *
 * PRI_LEG_A 映射到 EPWM9，是同步主模块。其它通道同步到 EPWM9 sync-out，并通过 TBPHS
 * 和同步后计数方向表达相对相位。
 */
static void Board_EPWM_configOne(Board_EPWMChannel ch)
{
    uint32_t base = Board_EPWM_getBase(ch);

    EPWM_setClockPrescaler(base, BOARD_EPWM_CLKDIV_ENUM, BOARD_EPWM_HSCLKDIV_ENUM);
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setTimeBaseCounter(base, 0U);
    EPWM_setPeriodLoadMode(base, EPWM_PERIOD_SHADOW_LOAD);
    EPWM_selectPeriodLoadEvent(base, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);
    EPWM_setTimeBasePeriod(base, gEpwmTbprd);
    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setActionQualifierActionComplete(base, EPWM_AQ_OUTPUT_A,
                                          BOARD_EPWM_AQ_ACTION_A);

    Board_EPWM_applyTripZoneSafeState(base);
    Board_EPWM_forceTrip(base);

    Board_EPWM_applyDuty(ch);
    Board_EPWM_applyPhase(ch);
    Board_EPWM_configDeadband(base);
    Board_EPWM_applyDeadband(ch);

    if (ch == BOARD_EPWM_PRI_LEG_A)
    {
        EPWM_setSyncInPulseSource(base, EPWM_SYNC_IN_PULSE_SRC_DISABLE);
        EPWM_disablePhaseShiftLoad(base);
    }
    else
    {
        EPWM_setSyncInPulseSource(base, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM9);
        EPWM_enablePhaseShiftLoad(base);
    }

    EPWM_enableSyncOutPulseSource(base, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);
}

/* Initialize all enabled EPWM modules.
 * 初始化所有启用的 EPWM 模块。
 *
 * Safety policy:
 * 1. Stop TBCLKSYNC before configuration.
 * 2. Configure all modules with duty=0.
 * 3. Force TripZone OST before and after TBCLKSYNC is enabled.
 *
 * 安全策略：
 * 1. 配置前关闭 TBCLKSYNC。
 * 2. 所有模块默认 duty=0。
 * 3. TBCLKSYNC 打开前后都保持 TripZone OST 关断。
 */
void Board_initEPWM(void)
{
    uint16_t i;

    gEpwmTbprd = Board_EPWM_calcTBPRD(BOARD_PWM_DEFAULT_FREQ_HZ);
    gEpwmDeadbandCount =
        Board_EPWM_calcDeadbandCount(BOARD_PWM_DEFAULT_DEADBAND_NS);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    for (i = 0U; i < (uint16_t)BOARD_EPWM_CHANNEL_COUNT; i++)
    {
        if (Board_EPWM_isChannelEnabled((Board_EPWMChannel)i))
        {
            gEpwm[i].duty = 0.0f;
            Board_EPWM_configOne((Board_EPWMChannel)i);
        }
    }

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    Board_EPWM_forceLowAll();
}

/* Force all enabled EPWM outputs low through TripZone one-shot.
 * 使用 TripZone one-shot 强制所有已启用 EPWM 输出为低。
 */
void Board_EPWM_forceLowAll(void)
{
    uint16_t i;

    for (i = 0U; i < (uint16_t)BOARD_EPWM_CHANNEL_COUNT; i++)
    {
        if (Board_EPWM_isChannelEnabled((Board_EPWMChannel)i))
        {
            Board_EPWM_forceTrip(Board_EPWM_getBase((Board_EPWMChannel)i));
        }
    }
}

/* Enable PWM outputs by clearing TripZone one-shot, only if no fault is latched.
 * 如果没有故障锁存，则清除 TripZone one-shot，允许 PWM 输出。
 */
void Board_EPWM_enableOutputs(void)
{
    uint16_t i;

    if (Board_SystemIsFault())
    {
        return;
    }

    for (i = 0U; i < (uint16_t)BOARD_EPWM_CHANNEL_COUNT; i++)
    {
        if (Board_EPWM_isChannelEnabled((Board_EPWMChannel)i))
        {
            Board_EPWM_clearTrip(Board_EPWM_getBase((Board_EPWMChannel)i));
        }
    }
}

/* Disable PWM outputs by re-forcing TripZone one-shot.
 * 通过重新触发 TripZone one-shot 禁止 PWM 输出。
 */
void Board_EPWM_disableOutputs(void)
{
    Board_EPWM_forceLowAll();
}

/* Configure EPWM9 ADCSOCA trigger source.
 * 配置 EPWM9 的 ADCSOCA 触发源。
 *
 * SOCA is generated once per PWM period at TBCTR=ZERO, which is the rising
 * edge of the EPWM9A AQ reference waveform in the current configuration.
 *
 * SOCA 在 TBCTR=ZERO 每个 PWM 周期产生一次；在当前 AQ 配置下，这就是
 * EPWM9A 参考波形的上升沿。
 *
 * This function only configures EPWM ADC trigger events. It does not configure
 * ADC SOC channels and does not enable SOCA.
 *
 * 本函数只配置 EPWM ADC 触发事件，不配置 ADC SOC 通道，也不使能 SOCA。
 */
void Board_EPWM_initADCTrigger(void)
{
    uint32_t base = BOARD_EPWM_PRI_LEG_A_BASE;

    EPWM_disableADCTrigger(base, EPWM_SOC_A);
    EPWM_disableADCTrigger(base, EPWM_SOC_B);

    /* EPWM9 SOCA is the only closed-loop ADC trigger.
     * EPWM9 SOCA 是闭环控制唯一 ADC 触发源。
     *
     * Per the current bring-up request, no blanking delay is inserted here:
     * SOCA is generated at CTR=ZERO, the EPWM9A rising edge, once per PWM
     * period. If delayed sampling is needed later, switch this source to
     * EPWM_SOC_TBCTR_U_CMPB and set CMPB.
     *
     * 按当前调试要求，这里不加入 blanking 延迟：SOCA 在 CTR=ZERO，即 EPWM9A
     * 上升沿，每个 PWM 周期触发一次。若后续需要延迟采样，再切换到
     * EPWM_SOC_TBCTR_U_CMPB 并设置 CMPB。
     */
    EPWM_setADCTriggerSource(base, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);

    EPWM_setADCTriggerEventPrescale(base, EPWM_SOC_A, 1U);

    EPWM_clearADCTriggerFlag(base, EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(base, EPWM_SOC_B);
}

/* Enable EPWM9 ADCSOCA trigger.
 * 使能 EPWM9 ADCSOCA 触发。
 */
void Board_EPWM_enableADCTrigger(void)
{
    uint32_t base = BOARD_EPWM_PRI_LEG_A_BASE;

    EPWM_clearADCTriggerFlag(base, EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(base, EPWM_SOC_B);

    EPWM_enableADCTrigger(base, EPWM_SOC_A);
    EPWM_disableADCTrigger(base, EPWM_SOC_B);
}

/* Disable EPWM9 ADC triggers.
 * 禁止 EPWM9 ADC 触发。
 */
void Board_EPWM_disableADCTrigger(void)
{
    uint32_t base = BOARD_EPWM_PRI_LEG_A_BASE;

    EPWM_disableADCTrigger(base, EPWM_SOC_A);
    EPWM_disableADCTrigger(base, EPWM_SOC_B);
}

/* Update PWM frequency while outputs may be active.
 * 在输出可能运行时更新 PWM 频率。
 *
 * Without Global Load, each EPWM shadow update may become active on its own
 * TBCTR=ZERO event. Multi-channel updates are not guaranteed to take effect in
 * exactly the same PWM cycle. For large frequency steps, use the safe variant.
 *
 * 未使用 Global Load 时，各路 EPWM shadow 更新会在各自 TBCTR=ZERO 生效，
 * 不能保证完全同一个 PWM 周期同步生效。大幅变频建议使用安全版本。
 */
void Board_EPWM_setFrequencyHz(uint32_t freq_hz)
{
    uint16_t i;

    if (freq_hz < BOARD_EPWM_MIN_FREQ_HZ)
    {
        freq_hz = BOARD_EPWM_MIN_FREQ_HZ;
    }
    else if (freq_hz > BOARD_EPWM_MAX_FREQ_HZ)
    {
        freq_hz = BOARD_EPWM_MAX_FREQ_HZ;
    }

    gBoardSystem.pwm_freq_hz = freq_hz;
    gEpwmTbprd = Board_EPWM_calcTBPRD(freq_hz);

    for (i = 0U; i < (uint16_t)BOARD_EPWM_CHANNEL_COUNT; i++)
    {
        if (Board_EPWM_isChannelEnabled((Board_EPWMChannel)i))
        {
            Board_EPWM_applyPeriod((Board_EPWMChannel)i);
            Board_EPWM_applyDuty((Board_EPWMChannel)i);
        }
    }

    Board_EPWM_applyAllPhase();
}

/* Safe frequency update: trip outputs first, then update frequency.
 * 安全变频：先 TripZone 关断输出，再更新频率。
 */
void Board_EPWM_setFrequencyHzSafe(uint32_t freq_hz)
{
    Board_EPWM_forceLowAll();
    Board_EPWM_setFrequencyHz(freq_hz);
}

/* Update deadband for all enabled channels.
 * 更新所有已启用通道的死区。
 */
void Board_EPWM_setDeadBandNs(uint16_t deadband_ns)
{
    uint16_t i;

    if (deadband_ns > BOARD_EPWM_MAX_DEADBAND_NS)
    {
        deadband_ns = BOARD_EPWM_MAX_DEADBAND_NS;
    }

    gBoardSystem.pwm_deadband_ns = deadband_ns;
    gEpwmDeadbandCount = Board_EPWM_calcDeadbandCount(deadband_ns);

    for (i = 0U; i < (uint16_t)BOARD_EPWM_CHANNEL_COUNT; i++)
    {
        if (Board_EPWM_isChannelEnabled((Board_EPWMChannel)i))
        {
            Board_EPWM_applyDeadband((Board_EPWMChannel)i);
        }
    }
}

/* Set duty for one logical channel.
 * 设置单个逻辑通道占空比。
 */
void Board_EPWM_setDuty(Board_EPWMChannel ch, float duty)
{
    if (!Board_EPWM_isChannelEnabled(ch))
    {
        return;
    }

    gEpwm[(uint16_t)ch].duty = Board_EPWM_clampDuty(duty);
    Board_EPWM_applyDuty(ch);
}

/* Set duty for all enabled channels.
 * 设置所有已启用通道占空比。
 */
void Board_EPWM_setDutyAll(float duty)
{
    uint16_t i;

    for (i = 0U; i < (uint16_t)BOARD_EPWM_CHANNEL_COUNT; i++)
    {
        Board_EPWM_setDuty((Board_EPWMChannel)i, duty);
    }
}

/* Set absolute phase in degrees.
 * 设置绝对相位，单位度。
 *
 * PRI_LEG_A is hardware master, but the user may still set its absolute phase.
 * All channels are then rebased relative to PRI_LEG_A internally.
 *
 * PRI_LEG_A 是硬件 master，但用户仍可设置它的绝对相位。
 * 内部会把所有通道重新换算成相对 PRI_LEG_A 的相位。
 */
void Board_EPWM_setPhaseDeg(Board_EPWMChannel ch, float phase_deg)
{
    if (!Board_EPWM_isChannelEnabled(ch))
    {
        return;
    }

    gEpwm[(uint16_t)ch].phase_deg_abs = Board_EPWM_normalizePhaseDeg(phase_deg);
    Board_EPWM_applyAllPhase();
}

/* Set absolute phase in radians.
 * 设置绝对相位，单位弧度。
 */
void Board_EPWM_setPhaseRad(Board_EPWMChannel ch, float phase_rad)
{
    Board_EPWM_setPhaseDeg(ch, phase_rad * 180.0f / BOARD_PI_F);
}

/* Set phase by raw count relative to PRI_LEG_A.
 * 通过原始计数设置相对 PRI_LEG_A 的相位。
 */
void Board_EPWM_setPhaseCount(Board_EPWMChannel ch, uint16_t phase_count)
{
    uint32_t modulo;
    uint16_t phase_count_max;
    float relative_deg;
    float master_phase;

    if (!Board_EPWM_isChannelEnabled(ch))
    {
        return;
    }

    modulo = (uint32_t)(2UL * (uint32_t)gEpwmTbprd);
    phase_count_max = PHASE_COUNT_MAX(gEpwmTbprd);

    /* Clamp the requested raw phase count before mapping it to TBPHS.
     * Board_EPWM_applyPhase() writes TBPHS; slave ePWM modules load TBPHS on
     * the next EPWM9 sync at CTR=ZERO, so ISR-side phase updates do not change
     * the active PWM edge in the middle of the current cycle.
     *
     * 写入前先限制原始移相计数。Board_EPWM_applyPhase() 写 TBPHS；从 ePWM
     * 在下一次 EPWM9 CTR=ZERO 同步点加载 TBPHS，因此 ISR 中更新移相不会在当前
     * 周期中途改变活动 PWM 边沿。
     */
    if (phase_count > phase_count_max)
    {
        phase_count = phase_count_max;
    }

    if (modulo != 0UL)
    {
        phase_count = (uint16_t)((uint32_t)phase_count % modulo);
    }

    relative_deg = ((float)phase_count * 360.0f) / (2.0f * (float)gEpwmTbprd);
    master_phase = gEpwm[(uint16_t)BOARD_EPWM_PRI_LEG_A].phase_deg_abs;
    gEpwm[(uint16_t)ch].phase_deg_abs =
        Board_EPWM_normalizePhaseDeg(master_phase + relative_deg);

    Board_EPWM_applyAllPhase();
}

/* Set all absolute phase values at once.
 * 一次设置四路绝对相位。
 */
void Board_EPWM_setAllPhaseDeg(float priA_deg,
                               float priB_deg,
                               float secA_deg,
                               float secB_deg)
{
    gEpwm[(uint16_t)BOARD_EPWM_PRI_LEG_A].phase_deg_abs =
        Board_EPWM_normalizePhaseDeg(priA_deg);
    gEpwm[(uint16_t)BOARD_EPWM_PRI_LEG_B].phase_deg_abs =
        Board_EPWM_normalizePhaseDeg(priB_deg);
    gEpwm[(uint16_t)BOARD_EPWM_SEC_LEG_A].phase_deg_abs =
        Board_EPWM_normalizePhaseDeg(secA_deg);
    gEpwm[(uint16_t)BOARD_EPWM_SEC_LEG_B].phase_deg_abs =
        Board_EPWM_normalizePhaseDeg(secB_deg);

    Board_EPWM_applyAllPhase();
}

/* Get current TBPRD.
 * 获取当前 TBPRD。
 */
uint16_t Board_EPWM_getTBPRD(void)
{
    return gEpwmTbprd;
}

/* Get current TBCLK in Hz.
 * 获取当前 TBCLK，单位 Hz。
 */
uint32_t Board_EPWM_getTBCLKHz(void)
{
    return BOARD_EPWM_TBCLK_HZ;
}
