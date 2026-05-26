/*------------------------------------------------------------------------------
 * DAB 双重双向调制优化控制算法 
 * 需注意公式的推导是用KCL/KVL近似出来的，主要体现在PN上面，只有当串联电容C>>谐振点，
 * 也就是纯的DAB控制时候才能用
 *
 * 按论文实现：
 *   - 输入 U1, U2, I2, D2_in
 *   - 计算 K = U1 / (n * U2)
 *   - 计算 PN = n * U1 * U2 / (8 * fsw * L)
 *   - 计算 p0 = |U2 * I2| / PN
 *   - 计算分段阈值 A, B, C
 *   - 按 K 与 p0 选择轨迹 R1 / R2 / R3 / R4
 *   - 由 D2 反解 D1
 *
 * 所有关键变量都做限幅 + NaN/Inf 检测，并把中间量写入 BDPS_OptResult_t。
 *----------------------------------------------------------------------------*/

#include "dab_bdps_opt.h"

/*==============================================================================
 * 文件结构
 *
 * 1. 硬件参数和调试结果全局变量
 * 2. 内部数学工具函数
 * 3. BDPS 算法主入口
 *============================================================================*/

/*==============================================================================
 * 1. 硬件参数和调试结果全局变量
 *============================================================================*/
volatile float32_t gBdpsN       = 4.0f;        /* 与 PrimaryTurn/SecondTurn=20/5=4 一致 */
volatile float32_t gBdpsLseries = 50.0e-6f;    /* 漏感+串联的储能电感，再次强调只能DAB用 */
volatile float32_t gBdpsFsw     = 100000.0f;   /* 100 kHz，与 BOARD_PWM_DEFAULT_FREQ_HZ 一致 */

BDPS_OptResult_t gBdpsLast = {0};

/* 0 < K < sqrt(3)/3 的分界点 */
#define BDPS_K_SPLIT  0.57735026918962576f  /* sqrt(3) / 3 */

/*==============================================================================
 * 2. 内部数学工具函数
 *============================================================================*/

/**
 * @brief 限幅工具函数。
 *
 * 输入：
 *   x: 原始输入值。
 *   lo / hi: 下限和上限。
 *
 * 输出：
 *   返回限制在 [lo, hi] 内的值。
 */
static inline float32_t bdps_clamp(float32_t x, float32_t lo, float32_t hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

/**
 * @brief 检查 NaN / Inf。
 *
 * 输入：
 *   x: 待检查浮点数。
 *
 * 输出：
 *   1: x 非法。
 *   0: x 正常。
 *
 * 说明：
 *   不依赖 math.h，避免与 driverlib 宏冲突。
 *   NaN 的性质是 x != x，Inf 用超大幅值近似判断。
 */
static inline int bdps_is_bad(float32_t x)
{
    return (x != x) || (x > 1.0e30f) || (x < -1.0e30f);
}

/*==============================================================================
 * 3. BDPS 算法主入口
 *============================================================================*/

/**
 * @brief 计算 BDPS 内移相 D1。
 *
 * 输入：
 *   U1: 原边电压，单位 V。
 *   U2: 副边电压，单位 V。
 *   I2: 副边电流，单位 A，允许为负。
 *   D2_in: 外移相 D2 输入，半周期归一化占空比。
 *   out: 结果结构体指针。
 *
 * 输出：
 *   out->D1: 内移相占空比。
 *   out->D2: 限幅后的外移相占空比。
 *   out->traj_id / err_flags: 轨迹和错误状态。
 */
void BDPS_CalcD1_Optimized(float32_t U1, float32_t U2, float32_t I2,
                           float32_t D2_in, BDPS_OptResult_t *out)
{
    float32_t n;
    float32_t K_raw, K;
    float32_t fsw, L, PN;
    float32_t Pout, p0_raw, p0;
    float32_t D2_raw, D2;
    float32_t one_minus_K, three_minus_K;
    float32_t A, B, C, C_num, C_den;
    float32_t D1, D1_raw;
    BDPS_Trajectory_t traj;

    if (out == 0)
    {
        return;
    }

    /* 默认结果清零，避免上一拍的中间量残留 */
    out->U1 = U1;
    out->U2 = U2;
    out->I2 = I2;
    out->K  = 0.0f;
    out->PN = 0.0f;
    out->Pout = 0.0f;
    out->p0 = 0.0f;
    out->A  = 0.0f;
    out->B  = 0.0f;
    out->C  = 0.0f;
    out->D2_in = D2_in;
    out->D2 = 0.0f;
    out->D1 = 0.0f;
    out->traj_id = BDPS_TRAJ_NONE;
    out->err_flags = BDPS_ERR_NONE;

    /* 1) U2 太小直接返回 D1=0，避免除零和数值奇异 */
    if (U2 < BDPS_U2_MIN_V)
    {
        out->err_flags |= BDPS_ERR_U2_TOO_SMALL;
        return;
    }

    /* 2) K = U1 / (n * U2)，限幅到 [0.001, 1.0] */
    n = gBdpsN;
    K_raw = U1 / (n * U2);
    K = bdps_clamp(K_raw, BDPS_K_MIN, BDPS_K_MAX);
    if (K != K_raw)
    {
        out->err_flags |= BDPS_ERR_K_CLAMPED;
    }
    out->K = K;

    /* 3) PN = n * U1 * U2 / (8 * fsw * L)
     * PN 是功率归一化的分母，物理上必为正。
     */
    fsw = gBdpsFsw;
    L   = gBdpsLseries;
    PN = (n * U1 * U2) / (8.0f * fsw * L);
    if (PN < BDPS_PN_MIN)
    {
        out->err_flags |= BDPS_ERR_PN_TOO_SMALL;
        out->PN = PN;
        return;
    }
    out->PN = PN;

    /* 4) Pout = U2 * I2，p0 = |Pout| / PN，限幅到 [0, 1]
     * 取绝对值：本算法只用幅值选轨迹，反向传输由 PWM 调制侧通过 D2 的极性体现。
     */
    Pout = U2 * I2;
    out->Pout = Pout;
    p0_raw = Pout / PN;
    if (p0_raw < 0.0f)
    {
        p0_raw = -p0_raw;
    }
    p0 = bdps_clamp(p0_raw, BDPS_P0_MIN, BDPS_P0_MAX);
    if (p0 != p0_raw)
    {
        out->err_flags |= BDPS_ERR_P0_CLAMPED;
    }
    out->p0 = p0;

    /* 5) D2 输入限幅 */
    D2_raw = D2_in;
    D2 = bdps_clamp(D2_raw, BDPS_D2_MIN, BDPS_D2_MAX);
    if (D2 != D2_raw)
    {
        out->err_flags |= BDPS_ERR_D2_CLAMPED;
    }
    out->D2 = D2;

    /* 6) 分段阈值 A, B, C
     *   A = (6 + 2K)(1 - K) / (3 - K)^2
     *   B = 1 - K^2
     *   C = (-K^2 + 2K + 3) / (K^2 + 2K + 3)
     */
    one_minus_K   = 1.0f - K;
    three_minus_K = 3.0f - K;
    A = ((6.0f + 2.0f * K) * one_minus_K) / (three_minus_K * three_minus_K);
    B = 1.0f - K * K;
    C_num = -K * K + 2.0f * K + 3.0f;
    C_den =  K * K + 2.0f * K + 3.0f;
    C = C_num / C_den;
    out->A = A;
    out->B = B;
    out->C = C;

    /* 7) 按 K 分两大分支，再按 p0 分段反解 D1
     *
     * R1: D2 = (1 - K + (K + 1) D1) / 2          -> D1 = (2D2 - 1 + K) / (K + 1)
     * R2: D2 = 2 D1                              -> D1 = D2 / 2
     * R3: D2 = (1 - K + (K + 3) D1) / 2          -> D1 = (2D2 - 1 + K) / (K + 3)
     * R4: D2 = 0.5 + K D1 / (K + 1)              -> D1 = (D2 - 0.5)(K + 1) / K
     *
     * K_min = 0.001，所以 K+1, K+3, K 均严格大于 0，无除零风险。
     */
    D1 = 0.0f;
    traj = BDPS_TRAJ_NONE;

    if (K < BDPS_K_SPLIT)
    {
        /* ---- 0 < K < sqrt(3)/3 ---- */
        if (p0 <= A)
        {
            traj = BDPS_TRAJ_R1;
            D1 = (2.0f * D2 - 1.0f + K) / (K + 1.0f);
        }
        else if (p0 <= B)
        {
            traj = BDPS_TRAJ_R1;
            D1 = (2.0f * D2 - 1.0f + K) / (K + 1.0f);
        }
        else if (p0 <= C)
        {
            traj = BDPS_TRAJ_R3;
            D1 = (2.0f * D2 - 1.0f + K) / (K + 3.0f);
        }
        else
        {
            traj = BDPS_TRAJ_R4;
            D1 = (D2 - 0.5f) * (K + 1.0f) / K;
        }
    }
    else
    {
        /* ---- sqrt(3)/3 <= K <= 1 ---- */
        if (p0 <= A)
        {
            traj = BDPS_TRAJ_R1;
            D1 = (2.0f * D2 - 1.0f + K) / (K + 1.0f);
        }
        else if (p0 <= B)
        {
            traj = BDPS_TRAJ_R2;
            D1 = D2 * 0.5f;
        }
        else if (p0 <= C)
        {
            traj = BDPS_TRAJ_R3;
            D1 = (2.0f * D2 - 1.0f + K) / (K + 3.0f);
        }
        else
        {
            traj = BDPS_TRAJ_R4;
            D1 = (D2 - 0.5f) * (K + 1.0f) / K;
        }
    }

    /* 8) NaN / Inf 检查 - 出现异常强制 D1=0 并置错误标志 */
    if (bdps_is_bad(D1))
    {
        out->err_flags |= BDPS_ERR_NAN_INF;
        D1 = 0.0f;
    }

    /* 9) D1 限幅 */
    D1_raw = D1;
    D1 = bdps_clamp(D1, BDPS_D1_MIN, BDPS_D1_MAX);
    if (D1 != D1_raw)
    {
        out->err_flags |= BDPS_ERR_D1_CLAMPED;
    }

    out->D1 = D1;
    out->traj_id = traj;
}
