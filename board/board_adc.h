#ifndef BOARD_ADC_H
#define BOARD_ADC_H

#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_ADC_DIFF_CHANNEL_COUNT    3U

/* Raw ADC result cache.
 * ADC 原始结果缓存。
 *
 * One sample group is triggered by EPWM9 SOCA at TBCTR=ZERO, the EPWM9A
 * rising edge, once per PWM period.
 *
 * 每个 PWM 周期由 EPWM9 SOCA 在 TBCTR=ZERO，也就是 EPWM9A 上升沿，
 * 触发一组采样。
 *
 * Array index:
 *   0 = C1 / D1
 *   1 = C2 / D2
 *   2 = C3 / D3
 *
 * 数组下标：
 *   0 = C1 / D1
 *   1 = C2 / D2
 *   2 = C3 / D3
 */
extern volatile int16_t gAdcCRaw[BOARD_ADC_DIFF_CHANNEL_COUNT];
extern volatile int16_t gAdcDRaw[BOARD_ADC_DIFF_CHANNEL_COUNT];

/* Normalized and ADC-pin voltage cache.
 * 归一化结果和 ADC 输入端电压缓存。
 *
 * Norm is approximately -1.0f~+1.0f for signed 16-bit differential results.
 * Volt is the differential voltage at the ADC input pins, not a power-stage
 * physical value such as vin, vout, or current.
 *
 * Norm 表示有符号 16-bit differential 结果的归一化值，约为 -1.0f~+1.0f。
 * Volt 表示 ADC 差分输入端电压，不是 vin、vout、电流等功率板物理量。
 */
extern volatile float gAdcCNorm[BOARD_ADC_DIFF_CHANNEL_COUNT];
extern volatile float gAdcDNorm[BOARD_ADC_DIFF_CHANNEL_COUNT];

extern volatile float gAdcCVolt[BOARD_ADC_DIFF_CHANNEL_COUNT];
extern volatile float gAdcDVolt[BOARD_ADC_DIFF_CHANNEL_COUNT];

/* Calibrated physical-value conversion.
 * 标定后的实际物理量转换。
 *
 * Actual = (Norm - Offset) * Scale.
 * 实际值 = (归一化值 - 偏置) * 系数。
 */
extern volatile float gAdcCActualOffset[BOARD_ADC_DIFF_CHANNEL_COUNT];
extern volatile float gAdcCActualScale[BOARD_ADC_DIFF_CHANNEL_COUNT];
extern volatile float gAdcDActualOffset[BOARD_ADC_DIFF_CHANNEL_COUNT];
extern volatile float gAdcDActualScale[BOARD_ADC_DIFF_CHANNEL_COUNT];

extern volatile float gAdcCActual[BOARD_ADC_DIFF_CHANNEL_COUNT];
extern volatile float gAdcDActual[BOARD_ADC_DIFF_CHANNEL_COUNT];

/* ADCD completion diagnostic flag.
 * ADCD 完成状态诊断标志。
 *
 * Set true when ADCC1 ISR fires before ADCD ADCINT1 is set.
 * 如果 ADCC1 ISR 进入时 ADCD ADCINT1 还没置位，则置 true。
 */
extern volatile bool gAdcDNotReady;

/* ADC analog pin placeholder.
 * ADC 模拟引脚占位初始化。
 */
void Board_ADC_initPins(void);

/* Initialize internal ADCC/ADCD for 16-bit differential sampling.
 * 初始化 C2000 内部 ADCC/ADCD，使用 16-bit differential 差分采样。
 */
void Board_initADC(void);

/* Copy ADCC/ADCD result registers into the raw cache.
 * 将 ADCC/ADCD 结果寄存器复制到原始缓存。
 */
void Board_ADC_UpdateCacheFromResult(void);

/* Convert cached raw signed ADC results to normalized and voltage values.
 * 将缓存的有符号 ADC 原始值转换为归一化值和电压值。
 */
void Board_ADC_ConvertCacheToFloat(void);

/* Convert normalized cache to calibrated physical values.
 * 将归一化缓存转换为标定后的实际物理量。
 */
void Board_ADC_ConvertNormToActual(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_ADC_H */
