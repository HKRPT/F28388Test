#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"

/*==============================================================================
 * 1. 通用宏定义
 *============================================================================*/

#define NULL_ADDR           0x00000000
#define SPS_PHASE_MAX_DEG   30.0f
#define EPS_PHASE_MAX_DEG   30.0f
#define EPS_CONSTANT(a,b,c) ((float32_t)((a) * (c) / (b)))
#define TPS_CONSTANT(a,b,c) ((float32_t)((a) * (c) / (b)))

/*==============================================================================
 * 2. 控制枚举定义
 *============================================================================*/

/**
 * @brief 功率传输方向。
 *
 * P2S: Primary to Secondary，能量从原边传到副边。
 * S2P: Secondary to Primary，能量从副边传到原边。
 */
enum CtrlLoopTransmissionMode
{
    P2S = 0,
    S2P
};

/**
 * @brief 控制层错误枚举。
 *
 * 当前主要作为错误类型预留，具体错误保护逻辑后续可以继续补充。
 * 注意：第一个故障枚举值为 0，因此判断是否故障要先看 sts == DABERROR，
 * 再读取 CTRLCSS.error 区分具体故障类型。
 */
enum CtrlLoopError
{
    PrimarySideTrankCurrentOverLoard = 0,
    SecondSideTrankCurrentOverLoard,
    PrimarySideInputCurrentOverLoard,
    SecondSideInputCurrentOverLoard,
    PrimarySideInputVoltageOverLoard,
    SecondSideInputVoltageOverLoard
};

/**
 * @brief 调制算法选择。
 *
 * SPSDAB: 单移相。
 * EPSDAB: 扩展移相。
 * TPSDAB: 三重移相，当前未实现。
 * BDPSOPTDAB: BDPS 优化调制。
 */
enum CtrlMode
{
    SPSDAB = 0,
    EPSDAB,
    TPSDAB,
    BDPSOPTDAB
};

/**
 * @brief DAB 控制状态机。
 *
 * DABREADYRUN: 准备运行或软启动中。
 * DABRUNING: 软启动完成，正常闭环运行。
 * DABWAITCHANGE: 目标值变化，等待 ISR 同步新目标。
 * DABERROR: 控制模式或方向非法。
 */
enum DABSTS
{
    DABREADYRUN = 0,
    DABRUNING,
    DABWAITCHANGE,
    DABERROR
};

/**
 * @brief 闭环模式。
 *
 * VMode: 单电压环。
 * IMode: 单电流环。
 * VIMode: 电压外环 + 电流内环。电压环是慢环，电流环是快环。
 */
enum LoopMode
{
    VMode = 0,
    IMode,
    VIMode
};

/**
 * @brief 软启动状态。
 */
enum SoftStaut
{
    SoftRunning = 0,
    SoftOver,
    SoftError
};

/*==============================================================================
 * 3. 控制结构体定义
 *============================================================================*/

/**
 * @brief 控制状态选择结构体。
 *
 * 输入：
 *   TargetVoltage / TargetCurrent 由临时目标值同步得到。
 *   CtrlMode / CtrlLoopTransmissionMode / LoopMode 由用户配置。
 *
 * 输出：
 *   sts 和 error 由控制状态机更新，用于判断当前运行状态和错误状态。
 */
typedef struct
{
    float32_t TargetVoltage;           /**< 目标电压，单位 V */
    float32_t TargetCurrent;           /**< 目标电流，单位 A */

    uint32_t sts;                      /**< 当前 DABSTS 状态  四个状态*/
    uint32_t error;                    /**< 控制错误标志，过流 */
    uint32_t CtrlMode;                 /**< 调制模式，取 enum CtrlMode */
    uint32_t CtrlLoopTransmissionMode; /**< 功率方向，取 enum CtrlLoopTransmissionMode */
    uint32_t LoopMode;                 /**< 闭环模式，取 enum LoopMode */
} CTRLCSS;

/**
 * @brief 软启动运行数据。
 *
 * 输入：
 *   SoftTime 为软启动总时间。
 *
 * 输出：
 *   SoftTemp / SoftOutput / SoftSTS 在软启动过程中实时更新。
 */
typedef struct
{
    uint16_t SoftSTS;        /**< 软启动状态，取 enum SoftStaut */

    float32_t SoftStep;      /**< 每个电流环周期增加的目标值 */
    float32_t SoftTarget;    /**< 软启动最终目标幅值 */
    float32_t SoftTime;      /**< 软启动时间，单位 s */
    float32_t SoftTemp;      /**< 当前软启动爬坡目标 */
    float32_t SoftOutput;    /**< 软启动阶段输出到调制的相角/占空比命令 */
} DAB_SoftStart_t;

/**
 * @brief DAB 过流/过压保护阈值。
 *
 * 输入：
 *   六个成员由用户按实际硬件额定值配置。
 *
 * 输出：
 *   ControlLoop_checkOverload() 读取这些阈值判断是否触发保护。
 *
 * 说明：
 *   电流类阈值按绝对值判断，电压类阈值按实际采样值大于阈值判断。
 */
typedef struct
{
    float32_t Overload_PrimarySide_TrankCurrent; /**< 原边槽路电流过流阈值，单位 A */
    float32_t Overload_SecondSide_TrankCurrent;  /**< 副边槽路电流过流阈值，单位 A */
    float32_t Overload_PrimarySide_InputCurrent; /**< 原边输入电流过流阈值，单位 A */
    float32_t Overload_SecondSide_InputCurrent;  /**< 副边输出电流过流阈值，单位 A */
    float32_t Overload_PrimarySide_Voltage;      /**< 原边电压过压阈值，单位 V */
    float32_t Overload_SecondSide_Voltage;       /**< 副边电压过压阈值，单位 V */
} DAB_OVERLOAD;

/**
 * @brief DAB 控制主对象。
 *
 * 输入：
 *   TargetVoltage / TargetCurrent 是当前闭环目标。
 *   CSS 指向状态选择结构体。
 *   SoftStar 指向软启动结构体。
 *
 * 输出：
 *   VoltageLoopOut 是电压环输出，VIMode 中作为电流参考。
 *   CurrentLoopOut 是电流环输出，通常作为最终调制输入。
 */
typedef struct
{
    float32_t TargetVoltage;     /**< 当前目标电压，单位 V */
    float32_t TargetCurrent;     /**< 当前目标电流，单位 A */

    float32_t VoltageLoopOut;    /**< 电压 PI 输出 */
    float32_t CurrentLoopOut;    /**< 电流 PI 输出 */

    CTRLCSS *CSS;                /**< 控制模式和状态指针 */
    DAB_SoftStart_t *SoftStar;   /**< 软启动数据指针 */
} CtrlLoop;

/*==============================================================================
 * 4. 默认初始化值
 *============================================================================*/

#define CTRLCSSDefaults  {0.0f, 0.0f, DABREADYRUN, 0U, SPSDAB, P2S, VMode}
#define CtrlLoopDefaults {0.0f, 0.0f, 0.0f, 0.0f, NULL_ADDR, NULL_ADDR}
#define DAB_OVERLOADDefaults {1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f}

/*==============================================================================
 * 5. 对外全局变量
 *============================================================================*/

/**
 * @brief DAB 过流/过压保护阈值。
 *
 * 输入：
 *   用户可在初始化代码或 CCS Expressions 中修改六个阈值。
 *
 * 输出：
 *   ADC ISR 中的保护检测函数读取该结构体，超限后触发 DABERROR 和软件 TZ。
 */
extern DAB_OVERLOAD DABOverload;

/*==============================================================================
 * 6. 对外函数声明
 *============================================================================*/

/**
 * @brief 初始化 DAB 控制层。
 *
 * 输入：无。
 * 输出：初始化全局 PI、控制状态、软启动状态。
 */
void ControlLoop_init(void);

/**
 * @brief ADC 中断中的主控制入口。
 *
 * 输入：ADC 采样结果来自 board_adc.c 的全局数组。
 * 输出：更新目标值、PI、软启动、主控制，并写入 PWM 相角。
 */
void ControlLoop_adcISR(void);

/**
 * @brief 慢任务入口。
 *
 * 输入：无。
 * 输出：当前为空，预留给 main while 中的低频任务。
 */
void ControlLoop_slowTask(void);

/**
 * @brief 请求更新电压环 PI 参数。
 *
 * 输入：读取 gControlVpiKp/Ki/Umax/Umin/Imax/Imin。
 * 输出：设置 VPI shadow 参数，等待 ISR 中 DCL_updatePI() 生效。
 */
void ControlLoop_requestVoltagePIUpdate(void);

/**
 * @brief 请求更新电流环 PI 参数。
 *
 * 输入：读取 gControlIpiKp/Ki/Umax/Umin/Imax/Imin。
 * 输出：设置 IPI shadow 参数，等待 ISR 中 DCL_updatePI() 生效。
 */
void ControlLoop_requestCurrentPIUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOOP_H */
