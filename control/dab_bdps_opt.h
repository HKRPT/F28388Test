#ifndef DAB_BDPS_OPT_H
#define DAB_BDPS_OPT_H

/*------------------------------------------------------------------------------
 * DAB BDPS 双重双向调制优化控制算法
 *
 * 选择优化轨迹（R1~R4），由外移相 D2 反解内移相 D1。
 *
 * 本模块只负责数学计算，不直接操作 ePWM 寄存器，也不依赖具体 ADC 索引。
 * D1 / D2 均为半周期归一化占空比，角度换算：deg = D * 180.0f。
 *----------------------------------------------------------------------------*/

#include <stdint.h>
#include "DCLF32.h"

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 * 轨迹编号 / Trajectory ID
 *----------------------------------------------------------------------------*/
typedef enum {
    BDPS_TRAJ_NONE = 0,
    BDPS_TRAJ_R1,
    BDPS_TRAJ_R2,
    BDPS_TRAJ_R3,
    BDPS_TRAJ_R4
} BDPS_Trajectory_t;

/*------------------------------------------------------------------------------
 * 错误标志位（按位 OR）
 *----------------------------------------------------------------------------*/
#define BDPS_ERR_NONE           0x0000U
#define BDPS_ERR_U2_TOO_SMALL   0x0001U   /* U2 低于阈值，直接返回 D1=0 */
#define BDPS_ERR_PN_TOO_SMALL   0x0002U   /* PN 太小，p0 无意义 */
#define BDPS_ERR_NAN_INF        0x0004U   /* 计算结果出现 NaN / Inf */
#define BDPS_ERR_K_CLAMPED      0x0008U   /* K 触发限幅 */
#define BDPS_ERR_P0_CLAMPED     0x0010U   /* p0 触发限幅 */
#define BDPS_ERR_D2_CLAMPED     0x0020U   /* D2 输入触发限幅 */
#define BDPS_ERR_D1_CLAMPED     0x0040U   /* D1 输出触发限幅 */

/*------------------------------------------------------------------------------
 * 调试结果结构体
 *
 * 所有中间量都保留出来，方便用 CCS Expressions 直接观察。
 *----------------------------------------------------------------------------*/
typedef struct {
    float32_t U1;                /* 原边电压采样 */
    float32_t U2;                /* 副边电压采样 */
    float32_t I2;                /* 副边电流采样 */
    float32_t K;                 /* 电压转换比 K = U1 / (n * U2) */
    float32_t PN;                /* 功率归一化系数 */
    float32_t Pout;              /* 实际输出功率 U2 * I2 */
    float32_t p0;                /* 传输功率标幺值 |Pout| / PN */
    float32_t A;                 /* 分段阈值 A = (6+2K)(1-K)/(3-K)^2 */
    float32_t B;                 /* 分段阈值 B = 1 - K^2 */
    float32_t C;                 /* 分段阈值 C = (-K^2+2K+3)/(K^2+2K+3) */
    float32_t D2_in;             /* 入参 D2（限幅前） */
    float32_t D2;                /* 限幅后的外移相占空比 */
    float32_t D1;                /* 反解出的内移相占空比 */
    BDPS_Trajectory_t traj_id;   /* 当前选中的轨迹编号 */
    uint16_t err_flags;          /* 错误/限幅标志位 */
} BDPS_OptResult_t;

/*------------------------------------------------------------------------------
 * 安全限值
 *
 * BDPS_D2_MAX 设为 0.5 是论文给定的外移相理论上限。
 * 如果实际系统希望更保守，可以在调用方再二次限幅。
 *----------------------------------------------------------------------------*/
#define BDPS_U2_MIN_V       1.0f
#define BDPS_K_MIN          0.001f
#define BDPS_K_MAX          1.0f
#define BDPS_P0_MIN         0.0f
#define BDPS_P0_MAX         1.0f
#define BDPS_D1_MIN         0.0f
#define BDPS_D1_MAX         1.0f
#define BDPS_D2_MIN         0.0f
#define BDPS_D2_MAX         0.5f
#define BDPS_PN_MIN         1.0e-6f

/*------------------------------------------------------------------------------
 * 硬件参数（占位 - 上板前必须按实际硬件确认）
 *
 * gBdpsN       : 变压器变比 n = N1 / N2，与 board_system.h 中 TurnsProportion 一致
 * gBdpsLseries : 等效串联电感 H（漏感 + 外加电感）
 * gBdpsFsw     : 开关频率 Hz，与 BOARD_PWM_DEFAULT_FREQ_HZ 一致
 *
 * 暴露为 volatile float32_t 全局，方便：
 *   1) 在 ControlLoop_init() 里覆盖默认值；
 *   2) 在调试时通过 CCS Expressions 直接改写。
 *
 * 若运行时调用 Board_EPWM_setFrequencyHz()，调用方需同步更新 gBdpsFsw。
 *----------------------------------------------------------------------------*/
extern volatile float32_t gBdpsN;
extern volatile float32_t gBdpsLseries;
extern volatile float32_t gBdpsFsw;

/*------------------------------------------------------------------------------
 * 全局结果（调试观察用）
 *----------------------------------------------------------------------------*/
extern BDPS_OptResult_t gBdpsLast;

/*------------------------------------------------------------------------------
 * 算法主入口
 *
 * 入参：
 *   U1     : 原边输入电压（V）
 *   U2     : 副边输出电压（V）
 *   I2     : 副边输出电流（A），允许负值（反向传输）
 *   D2_in  : 外环 PI 给出的外移相占空比（0~0.5）
 *   out    : 输出结构体指针，函数内部完整填充
 *
 * 出参：out->D1 即可直接用于 PWM 相位生成。
 *----------------------------------------------------------------------------*/
void BDPS_CalcD1_Optimized(float32_t U1, float32_t U2, float32_t I2,
                           float32_t D2_in, BDPS_OptResult_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DAB_BDPS_OPT_H */
