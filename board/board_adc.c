#include "board/board_adc.h"

/* ADC acquisition window.
 * ADC 采样窗口。
 *
 * 16-bit differential sampling usually needs a longer acquisition window than
 * 12-bit single-ended sampling. Keep this conservative first, then tune it
 * later according to ADC clock and signal source impedance.
 *
 * 16-bit differential 差分采样通常比 12-bit single-ended 单端采样需要更长
 * 的采样窗口。这里先保守设置，后续再按 ADC 时钟和信号源阻抗调整。
 */
#define BOARD_ADC_ACQPS                 64U

/* Differential conversion scale.
 * 差分转换比例。
 *
 * BOARD_ADC_DIFF_FULL_SCALE_V is a placeholder for the ADC differential input
 * full-scale voltage. Confirm it later from the F28388D ADC reference setting
 * and board hardware reference voltage. Do not assume the final value is 3.0 V
 * or 3.3 V without measurement and datasheet confirmation.
 *
 * BOARD_ADC_DIFF_FULL_SCALE_V 是 ADC 差分输入满量程电压占位值。后续需要根据
 * F28388D ADC reference 配置和硬件参考电压确认，不能直接假定最终一定是
 * 3.0 V 或 3.3 V。
 *
 * Current interpretation assumes the 16-bit differential result can be treated
 * as signed int16_t. If 0 V differential input later reads near 0x8000 instead
 * of near 0, revisit the output format interpretation.
 *
 * 当前按 16-bit differential 结果可解释为 int16_t 处理。如果后续实测 0 V
 * 差分输入接近 0x8000 而不是接近 0，需要重新检查输出格式解释。
 */
#define BOARD_ADC_DIFF_FULL_SCALE_V     3.0f
#define BOARD_ADC_DIFF_CODE_SCALE       32768.0f

/* Placeholder differential channel mapping.
 * 差分通道占位映射。
 *
 * These macros are only placeholders. Confirm the real ADC_CH_* differential
 * pair for DSP_ADC_C_1_P/N, C_2_P/N, C_3_P/N and DSP_ADC_D_1_P/N, D_2_P/N,
 * D_3_P/N from the schematic before assigning physical meanings.
 *
 * 下面只是占位宏。后续必须按原理图确认 DSP_ADC_C_1_P/N、C_2_P/N、C_3_P/N
 * 和 DSP_ADC_D_1_P/N、D_2_P/N、D_3_P/N 对应的真实 ADC_CH_* 差分对，
 * 不要默认认为这些占位值就是某个具体物理量。
 */
#define BOARD_ADCC_C1_CHANNEL           ADC_CH_ADCIN0_ADCIN1
#define BOARD_ADCC_C2_CHANNEL           ADC_CH_ADCIN2_ADCIN3
#define BOARD_ADCC_C3_CHANNEL           ADC_CH_ADCIN4_ADCIN5

#define BOARD_ADCD_D1_CHANNEL           ADC_CH_ADCIN0_ADCIN1
#define BOARD_ADCD_D2_CHANNEL           ADC_CH_ADCIN2_ADCIN3
#define BOARD_ADCD_D3_CHANNEL           ADC_CH_ADCIN4_ADCIN5

volatile int16_t gAdcCRaw[BOARD_ADC_DIFF_CHANNEL_COUNT] = {0};
volatile int16_t gAdcDRaw[BOARD_ADC_DIFF_CHANNEL_COUNT] = {0};

volatile float gAdcCNorm[BOARD_ADC_DIFF_CHANNEL_COUNT] = {0.0f};
volatile float gAdcDNorm[BOARD_ADC_DIFF_CHANNEL_COUNT] = {0.0f};

volatile float gAdcCVolt[BOARD_ADC_DIFF_CHANNEL_COUNT] = {0.0f};
volatile float gAdcDVolt[BOARD_ADC_DIFF_CHANNEL_COUNT] = {0.0f};

volatile float gAdcCActualOffset[BOARD_ADC_DIFF_CHANNEL_COUNT] = {0.0f};
volatile float gAdcCActualScale[BOARD_ADC_DIFF_CHANNEL_COUNT]  = {1.0f, 1.0f, 1.0f};
volatile float gAdcDActualOffset[BOARD_ADC_DIFF_CHANNEL_COUNT] = {0.0f};
volatile float gAdcDActualScale[BOARD_ADC_DIFF_CHANNEL_COUNT]  = {1.0f, 1.0f, 1.0f};

volatile float gAdcCActual[BOARD_ADC_DIFF_CHANNEL_COUNT] = {0.0f};
volatile float gAdcDActual[BOARD_ADC_DIFF_CHANNEL_COUNT] = {0.0f};

volatile bool gAdcDNotReady = false;

/* Convert signed raw ADC result to normalized value.
 * 将有符号 ADC 原始值转换为归一化值。
 */
static float Board_ADC_rawToNorm(int16_t raw)
{
    return ((float)raw) / BOARD_ADC_DIFF_CODE_SCALE;
}

/* Convert signed raw ADC result to ADC differential input voltage.
 * 将有符号 ADC 原始值转换为 ADC 差分输入端电压。
 */
static float Board_ADC_rawToVoltage(int16_t raw)
{
    return Board_ADC_rawToNorm(raw) * BOARD_ADC_DIFF_FULL_SCALE_V;
}

/* Convert normalized ADC value to calibrated physical value.
 * 将归一化 ADC 值转换为标定后的实际物理值。
 */
static float Board_ADC_normToActual(float norm, float offset, float scale)
{
    return (norm - offset) * scale;
}

/* Configure common ADC module settings.
 * 配置单个 ADC 模块的公共参数。
 */
static void Board_ADC_configModule(uint32_t base)
{
    ADC_setPrescaler(base, ADC_CLK_DIV_4_0);
    ADC_setMode(base, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
    ADC_setInterruptPulseMode(base, ADC_PULSE_END_OF_CONV);
    ADC_enableConverter(base);
}

/* ADC analog input pin placeholder.
 * ADC 模拟输入脚占位。
 *
 * ADCC / ADCD differential analog inputs:
 * C1/C2/C3 and D1/D2/D3.
 *
 * If these pins are dedicated ADCIN pins, no GPIO PinMux configuration is
 * required. If they are ADC-capable GPIO / AGPIO pins, enable analog mode here
 * after checking the F28388D datasheet and schematic.
 *
 * Do not configure ADC pins as normal GPIO output. Do not call GPIO_writePin()
 * for ADC pins.
 *
 * ADCC / ADCD 差分模拟输入包括 C1/C2/C3 和 D1/D2/D3。
 *
 * 如果这些引脚是专用 ADCIN 模拟输入脚，则不需要 GPIO PinMux 配置。如果它们是
 * ADC-capable GPIO / AGPIO，则必须查 F28388D datasheet 和原理图后，再在这里
 * 使能 analog mode。
 *
 * 不要把 ADC 引脚配置成普通 GPIO 输出，也不要对 ADC 引脚调用 GPIO_writePin()。
 */
void Board_ADC_initPins(void)
{
}

/* Initialize ADCC/ADCD SOCs and ADCC1 interrupt source.
 * 初始化 ADCC/ADCD 的 SOC，以及 ADCC1 中断源。
 */
void Board_initADC(void)
{
    Board_ADC_initPins();

    Board_ADC_configModule(ADCC_BASE);
    Board_ADC_configModule(ADCD_BASE);

    /* Wait for ADC analog circuits to settle after converter enable.
     * ADC converter 使能后等待模拟电路稳定。
     */
    DEVICE_DELAY_US(1000U);

    /* ADCC SOC0~SOC2: all control conversions are triggered once per PWM
     * period by EPWM9 SOCA at the EPWM9A rising edge. ADCINT1 is generated
     * only after the last control SOC, SOC2, finishes.
     *
     * ADCC SOC0~SOC2：所有闭环采样均由 EPWM9 SOCA 在 EPWM9A 上升沿、
     * 每个 PWM 周期触发一次。仅最后一个控制相关 SOC2 完成后产生 ADCINT1。
     */
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM9_SOCA,
                 BOARD_ADCC_C1_CHANNEL, BOARD_ADC_ACQPS);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM9_SOCA,
                 BOARD_ADCC_C2_CHANNEL, BOARD_ADC_ACQPS);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM9_SOCA,
                 BOARD_ADCC_C3_CHANNEL, BOARD_ADC_ACQPS);

    /* ADCD SOC0~SOC2: same EPWM9 SOCA trigger as ADCC.
     * ADCD SOC0~SOC2：与 ADCC 一样使用 EPWM9 SOCA 触发。
     */
    ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM9_SOCA,
                 BOARD_ADCD_D1_CHANNEL, BOARD_ADC_ACQPS);
    ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM9_SOCA,
                 BOARD_ADCD_D2_CHANNEL, BOARD_ADC_ACQPS);
    ADC_setupSOC(ADCD_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM9_SOCA,
                 BOARD_ADCD_D3_CHANNEL, BOARD_ADC_ACQPS);

    /* ADCC1 ISR is generated after ADCC SOC2 EOC.
     * ADCC SOC2 转换完成后产生 ADCC1 中断。
     */
    ADC_disableContinuousMode(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);
    ADC_clearInterruptOverflowStatus(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);

    /* ADCD ADCINT1 is also sourced from SOC2 EOC so its completion flag can be
     * checked in ADCC1 ISR. INT_ADCD1 is not registered to the CPU at this
     * stage; ADCC1 remains the only ADC ISR entry.
     *
     * ADCD ADCINT1 也由 SOC2 EOC 置位，这样可以在 ADCC1 ISR 中检查 ADCD
     * 是否也完成。当前阶段不注册 INT_ADCD1 到 CPU，ADC 中断入口仍只有 ADCC1。
     */
    ADC_disableContinuousMode(ADCD_BASE, ADC_INT_NUMBER1);
    ADC_setInterruptSource(ADCD_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);
    ADC_clearInterruptOverflowStatus(ADCD_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
    ADC_enableInterrupt(ADCD_BASE, ADC_INT_NUMBER1);
}

/* Read raw conversion results into cache.
 * 读取原始转换结果到缓存。
 *
 * Use ADCCRESULT_BASE / ADCDRESULT_BASE for result reads, not ADCC_BASE /
 * ADCD_BASE.
 *
 * 读取结果必须使用 ADCCRESULT_BASE / ADCDRESULT_BASE，不使用 ADCC_BASE /
 * ADCD_BASE。
 */
void Board_ADC_UpdateCacheFromResult(void)
{
    gAdcCRaw[0U] = (int16_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
    gAdcCRaw[1U] = (int16_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);
    gAdcCRaw[2U] = (int16_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2);

    gAdcDRaw[0U] = (int16_t)ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0);
    gAdcDRaw[1U] = (int16_t)ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER1);
    gAdcDRaw[2U] = (int16_t)ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER2);

    Board_ADC_ConvertCacheToFloat();
}




/* Convert raw cache to normalized and voltage cache.
 * 将 raw 缓存转换为归一化值和电压缓存。
 */
void Board_ADC_ConvertCacheToFloat(void)
{
    uint16_t i;

    for (i = 0U; i < BOARD_ADC_DIFF_CHANNEL_COUNT; i++)
    {
        gAdcCNorm[i] = Board_ADC_rawToNorm(gAdcCRaw[i]);
        gAdcDNorm[i] = Board_ADC_rawToNorm(gAdcDRaw[i]);

        gAdcCVolt[i] = Board_ADC_rawToVoltage(gAdcCRaw[i]);
        gAdcDVolt[i] = Board_ADC_rawToVoltage(gAdcDRaw[i]);

    }

    Board_ADC_ConvertNormToActual();
}

/* Convert normalized cache to calibrated physical-value cache.
 * 将归一化缓存转换为标定后的实际物理量缓存。
 */
void Board_ADC_ConvertNormToActual(void)
{
    uint16_t i;

    for (i = 0U; i < BOARD_ADC_DIFF_CHANNEL_COUNT; i++)
    {
        gAdcCActual[i] = Board_ADC_normToActual(gAdcCNorm[i],
                                                gAdcCActualOffset[i],
                                                gAdcCActualScale[i]);
        gAdcDActual[i] = Board_ADC_normToActual(gAdcDNorm[i],
                                                gAdcDActualOffset[i],
                                                gAdcDActualScale[i]);
    }
}
