#include "ControlLoop.h"
#include "board.h"
#include "DCL.h"
#include "DCLF32.h"
#include "dab_bdps_opt.h"

/*==============================================================================
 * 文件结构
 *
 * 1. 外部采样数组声明
 * 2. 控制周期、PI 参数、目标值等全局变量
 * 3. 内部工具函数
 * 4. 对外参数更新函数
 * 5. 控制初始化函数
 * 6. 调制输出函数
 * 7. 软启动、主控制和 ADC ISR
 *============================================================================*/

/*------------------------------------------------------------------------------
 * 1. 外部采样数组声明
 *
 * 输入来源：board_adc.c 中 ADC 换算后的实际物理量。
 * 输出用途：本文件只读取，不在此处写入。
 *----------------------------------------------------------------------------*/
extern volatile float gAdcCActual[BOARD_ADC_DIFF_CHANNEL_COUNT];
extern volatile float gAdcDActual[BOARD_ADC_DIFF_CHANNEL_COUNT];

/*
 * ADC 下标约定：
 *   gAdcCActual[0] = Second Side Current        副边输出电流
 *   gAdcCActual[1] = Second Side Tank Current   副边槽路电流
 *   gAdcCActual[2] = Second Side Voltage        副边电压
 *   gAdcDActual[0] = Primary Side Tank Current  原边槽路电流
 *   gAdcDActual[1] = Primary Side Voltage       原边电压
 *   gAdcDActual[2] = Primary Side Current       原边输入电流
 */

/*==============================================================================
 * 2. 控制周期、PI 参数、目标值等全局变量
 *============================================================================*/

/* Current loop runs in ADC ISR. Confirm this value from the final ePWM period.
 * 电流环在 ADC ISR 中运行；这个周期后续需要按最终 ePWM 周期确认。
 * 当前 ADC 由 EPWM9 SOCA 在 EPWM9A 上升沿、每个 PWM 周期触发一次，控制周期等于 PWM 周期。
 */
volatile float32_t gControlCurrentLoopTsSec = 10.0e-6f;


/* Voltage loop is normally slower than current loop.
 * 电压外环通常比电流内环慢，先按 10 分频占位。
 */
volatile uint16_t gControlVoltageLoopDiv = 10U;
/*
 *此处为电压环的控制周期
 */
volatile float32_t gControlVoltageLoopTsSec = 100.0e-6f;

/* Placeholder PI gains and limits.
 * PI初始化值 。
 */
 //PI初始化
volatile float32_t gControlVpiKp = 0.01f;
volatile float32_t gControlVpiKi = 0.001f;
//电压环输出极限
volatile float32_t gControlVpiUmax = 10.0f;
volatile float32_t gControlVpiUmin = 0.0f;
//电压环积分饱和极限
volatile float32_t gControlVpiImax = 10.0f;
volatile float32_t gControlVpiImin = 0.0f;
//PI初始化
volatile float32_t gControlIpiKp = 0.01f;
volatile float32_t gControlIpiKi = 0.001f;
//电流环输出饱和极限
volatile float32_t gControlIpiUmax = 1.0f;
volatile float32_t gControlIpiUmin = -1.0f;
//积分饱和极限
volatile float32_t gControlIpiImax = 1.0f;
volatile float32_t gControlIpiImin = -1.0f;

volatile float32_t TempTargetVoltage;
volatile float32_t TempTargetCurrent;

/*
 * 电压环分频计数器：
 * gControlVoltageLoopDiv = 10 时，电压 PI 每 10 个电流环周期更新一次；
 * 电流 PI 仍然每个 ADC ISR 都运行，使用最近一次电压环输出作为电流参考。
 */
static uint16_t gControlVoltageLoopCnt = 0U;


CtrlLoop DABCtrl =  CtrlLoopDefaults;
CTRLCSS  DABCSS = CTRLCSSDefaults;
DAB_SoftStart_t  DABSoftStart;
DAB_OVERLOAD DABOverload = DAB_OVERLOADDefaults;

DCL_PI VPI = PI_DEFAULTS;    //这里初始化PI结构体，DCL的结构体里面自己有一个默认的初始化值
DCL_PI_SPS VPI_SPS = PI_SPS_DEFAULTS;//影子结构体,DCL的结构体里面自己有一个默认的初始化值
DCL_CSS VPI_CSS = DCL_CSS_DEFAULTS;//默认的结构体，处理报错等信息,DCL的结构体里面自己有一个默认的初始化值
//下面同理上面
DCL_PI IPI = PI_DEFAULTS;
DCL_PI_SPS IPI_SPS = PI_SPS_DEFAULTS;
DCL_CSS IPI_CSS = DCL_CSS_DEFAULTS;

/*==============================================================================
 * 3. 内部工具函数
 *============================================================================*/

/**
 * @brief 初始化一个 DCL PI 控制器。
 *
 * 输入：
 *   pi / sps / css: DCL PI 主结构体、影子参数、公共状态。
 *   periodSec: 控制周期，单位 s。
 *   kp / ki: 连续域 PI 参数。
 *   umax / umin: PI 输出限幅。
 *   imax / imin: PI 积分限幅。
 *
 * 输出：
 *   写入 DCL 结构体参数，并清空 PI 积分状态。
 */
static void ControlLoop_initPI(DCL_PI *pi, DCL_PI_SPS *sps, DCL_CSS *css,
                               float32_t periodSec, float32_t kp, float32_t ki,
                               float32_t umax, float32_t umin,
                               float32_t imax, float32_t imin)
{
    pi->sps = sps;
    pi->css = css;

    DCL_SET_CONTROLLER_PERIOD(pi, periodSec);

    sps->Kp = kp;
    sps->Ki = ki*periodSec;//连续转离散
    sps->Umax = umax;
    sps->Umin = umin;
    sps->Imax = imax;
    sps->Imin = imin;

    DCL_REQUEST_UPDATE(pi);
    DCL_updatePI(pi);
    DCL_resetPI(pi);
}

/**
 * @brief 请求更新一个 DCL PI 控制器参数。
 *
 * 输入：
 *   pi / sps: 待更新的 PI 控制器和影子参数。
 *   kp / ki / umax / umin / imax / imin: 新 PI 参数。
 *
 * 输出：
 *   写入 shadow 参数，并置 DCL_REQUEST_UPDATE，实际更新在 ISR 中完成。
 */
static void ControlLoop_requestPIUpdate(DCL_PI *pi, DCL_PI_SPS *sps,
                                        float32_t kp, float32_t ki,
                                        float32_t umax, float32_t umin,
                                        float32_t imax, float32_t imin)
{
    sps->Kp = kp;
    sps->Ki = ki * pi->css->T;//连续转离散
    sps->Umax = umax;
    sps->Umin = umin;
    sps->Imax = imax;
    sps->Imin = imin;

    DCL_REQUEST_UPDATE(pi);
}

/*==============================================================================
 * 4. 对外参数更新函数
 *============================================================================*/

/**
 * @brief 请求更新电压环 PI 参数。
 *
 * 输入：
 *   gControlVpiKp/Ki/Umax/Umin/Imax/Imin 和 gControlVoltageLoopDiv。
 *
 * 输出：
 *   更新电压环控制周期和 shadow 参数，等待 ADC ISR 中 DCL_updatePI() 生效。
 */
void ControlLoop_requestVoltagePIUpdate(void)//更新电压环
{
    gControlVoltageLoopTsSec = gControlCurrentLoopTsSec *
                               (float32_t)((gControlVoltageLoopDiv == 0U) ? 1U :
                                            gControlVoltageLoopDiv);
    DCL_SET_CONTROLLER_PERIOD(&VPI, gControlVoltageLoopTsSec);

    ControlLoop_requestPIUpdate(&VPI, &VPI_SPS,
                                gControlVpiKp, gControlVpiKi,
                                gControlVpiUmax, gControlVpiUmin,
                                gControlVpiImax, gControlVpiImin);
}

/**
 * @brief 请求更新电流环 PI 参数。
 *
 * 输入：
 *   gControlIpiKp/Ki/Umax/Umin/Imax/Imin。
 *
 * 输出：
 *   更新电流环 shadow 参数，等待 ADC ISR 中 DCL_updatePI() 生效。
 */
void ControlLoop_requestCurrentPIUpdate(void)//更新电流环
{
    ControlLoop_requestPIUpdate(&IPI, &IPI_SPS,
                                gControlIpiKp, gControlIpiKi,
                                gControlIpiUmax, gControlIpiUmin,
                                gControlIpiImax, gControlIpiImin);
}

/**
 * @brief 同时请求更新电压环和电流环 PI 参数。
 *
 * 输入：所有电压环、电流环 PI 参数全局变量。
 * 输出：两个 PI 控制器都将在 ISR 中更新。
 */
void ControlLoop_requestAllPIUpdate(void)
{
    ControlLoop_requestVoltagePIUpdate();
    ControlLoop_requestCurrentPIUpdate();
}

/**
 * @brief 计算 float32_t 绝对值。
 *
 * 输入：x。
 * 输出：|x|。
 */
static float32_t ControlLoop_absF32(float32_t x)
{
    return (x < 0.0f) ? -x : x;
}

/**
 * @brief 复位电压外环分频计数器。
 *
 * 输入：无。
 * 输出：下一次调用 ControlLoop_runVoltagePI() 时立即运行一次电压 PI。
 */
static void ControlLoop_resetVoltageLoopDivider(void)
{
    gControlVoltageLoopCnt = 0U;
}

/**
 * @brief 判断当前拍是否应该运行电压 PI。
 *
 * 输入：
 *   gControlVoltageLoopDiv，表示电压环相对电流环的分频。
 *
 * 输出：
 *   1: 本拍运行电压 PI。
 *   0: 本拍保持上一次电压 PI 输出。
 */
static uint16_t ControlLoop_isVoltageLoopUpdateTime(void)
{
    uint16_t div = gControlVoltageLoopDiv;

    if (div <= 1U)
    {
        return 1U;
    }

    /*
     * 计数器为 0 时立即运行一次电压环，保证刚进入软启动/运行时
     * VoltageLoopOut 不是旧值；随后保持 div 个电流环周期再更新。
     */
    if (gControlVoltageLoopCnt == 0U)
    {
        gControlVoltageLoopCnt = div - 1U;
        return 1U;
    }

    gControlVoltageLoopCnt--;
    return 0U;
}

/**
 * @brief 按分频节奏运行电压 PI。
 *
 * 输入：
 *   C: 控制对象。
 *   ref: 电压参考值。
 *   feedback: 电压反馈值。
 *
 * 输出：
 *   C->VoltageLoopOut 被周期性更新；未到周期时保持旧值。
 *   返回当前 VoltageLoopOut。
 */
static float32_t ControlLoop_runVoltagePI(CtrlLoop *C,
                                          float32_t ref,
                                          float32_t feedback)
{
    if (ControlLoop_isVoltageLoopUpdateTime())
    {
        C->VoltageLoopOut = DCL_runPI_C3(&VPI, ref, feedback);
    }

    return C->VoltageLoopOut;
}

/**
 * @brief 按闭环模式清空相关 PI 积分。
 *
 * 输入：
 *   loopMode: VMode/IMode/VIMode。
 *
 * 输出：
 *   清空对应 PI 积分，并复位电压环分频计数器。
 */
static void ControlLoop_resetPIByMode(uint32_t loopMode)
{
    if (loopMode == VMode)
    {
        DCL_resetPI(&VPI);
    }
    else if (loopMode == IMode)
    {
        DCL_resetPI(&IPI);
    }
    else if (loopMode == VIMode)
    {
        DCL_resetPI(&VPI);
        DCL_resetPI(&IPI);
    }

    ControlLoop_resetVoltageLoopDivider();
}

/**
 * @brief 触发 DAB 控制层保护。
 *
 * 输入：
 *   error: enum CtrlLoopError 中的故障类型。
 *
 * 输出：
 *   写入 DABCSS.error，状态切到 DABERROR，并通过软件 TZ 拉闸 PWM。
 */
static void ControlLoop_tripFault(uint32_t error)
{
    DABCSS.error = error;
    DABCSS.sts = DABERROR;

    if (DABCtrl.CSS != 0)
    {
        DABCtrl.CSS->error = error;
        DABCtrl.CSS->sts = DABERROR;
    }

    Board_SystemSetFault();
    Board_EPWM_forceLowAll();
}

/**
 * @brief 检查六个 ADC 实际采样值是否过流/过压。
 *
 * 输入：
 *   使用 gAdcCActual[] / gAdcDActual[] 六个实际采样值。
 *   使用 DABOverload 中配置的六个保护阈值。
 *
 * 输出：
 *   0: 未触发保护。
 *   1: 已触发保护，DABCSS.error 写入具体 enum CtrlLoopError，
 *      DABCSS.sts 切到 DABERROR，并触发软件 TripZone。
 */
static uint16_t ControlLoop_checkOverload(void)
{
    if (ControlLoop_absF32(gAdcDActual[0]) > DABOverload.Overload_PrimarySide_TrankCurrent)
    {
        ControlLoop_tripFault(PrimarySideTrankCurrentOverLoard);
        return 1U;
    }

    if (ControlLoop_absF32(gAdcCActual[1]) > DABOverload.Overload_SecondSide_TrankCurrent)
    {
        ControlLoop_tripFault(SecondSideTrankCurrentOverLoard);
        return 1U;
    }

    if (ControlLoop_absF32(gAdcDActual[2]) > DABOverload.Overload_PrimarySide_InputCurrent)
    {
        ControlLoop_tripFault(PrimarySideInputCurrentOverLoard);
        return 1U;
    }

    if (ControlLoop_absF32(gAdcCActual[0]) > DABOverload.Overload_SecondSide_InputCurrent)
    {
        ControlLoop_tripFault(SecondSideInputCurrentOverLoard);
        return 1U;
    }

    if (gAdcDActual[1] > DABOverload.Overload_PrimarySide_Voltage)
    {
        ControlLoop_tripFault(PrimarySideInputVoltageOverLoard);
        return 1U;
    }

    if (gAdcCActual[2] > DABOverload.Overload_SecondSide_Voltage)
    {
        ControlLoop_tripFault(SecondSideInputVoltageOverLoard);
        return 1U;
    }

    return 0U;
}

/**
 * @brief 配置一轮软启动参数。
 *
 * 输入：
 *   C: 控制对象，读取目标值、闭环模式和 SoftTime。
 *
 * 输出：
 *   计算 SoftTarget、SoftStep，复位 SoftTemp/SoftOutput，并清 PI 积分。
 */
static void ControlLoop_configSoftStart(CtrlLoop *C)
{
    float32_t target;

    if ((C == 0) || (C->CSS == 0) || (C->SoftStar == 0))
    {
        return;
    }

    /* 软启动只按幅值爬坡，功率方向在 DABSoftStar() 中由 P2S/S2P 决定。 */
    if (C->CSS->LoopMode == IMode)
    {
        target = ControlLoop_absF32(C->TargetCurrent);
    }
    else
    {
        target = ControlLoop_absF32(C->TargetVoltage);
    }

    C->SoftStar->SoftTarget = target;
    C->SoftStar->SoftTemp = 0.0f;
    C->SoftStar->SoftOutput = 0.0f;

    /* 用实际电流环周期计算每拍步长，避免硬编码 100 kHz。 */
    if ((target <= 0.0f) || (C->SoftStar->SoftTime <= 0.0f) ||
        (gControlCurrentLoopTsSec <= 0.0f))
    {
        C->SoftStar->SoftStep = target;
        C->SoftStar->SoftSTS = SoftOver;
    }
    else
    {
        C->SoftStar->SoftStep = target * gControlCurrentLoopTsSec /
                                 C->SoftStar->SoftTime;
        C->SoftStar->SoftSTS = SoftRunning;
    }

    /* 新一轮软启动从干净 PI 状态开始，避免旧积分把相角顶到饱和。 */
    ControlLoop_resetPIByMode(C->CSS->LoopMode);
}



/*==============================================================================
 * 5. 控制初始化函数
 *============================================================================*/

/**
 * @brief 初始化控制层全局对象、PI 控制器和软启动。
 *
 * 输入：
 *   读取 TempTargetVoltage / TempTargetCurrent 作为初始目标。
 *
 * 输出：
 *   初始化 DABCtrl、DABCSS、DABSoftStart、VPI、IPI。
 */
void ControlLoop_init(void)//中断初始化
{
    gControlVoltageLoopTsSec = gControlCurrentLoopTsSec *
                               (float32_t)((gControlVoltageLoopDiv == 0U) ? 1U :
                                            gControlVoltageLoopDiv);

    ControlLoop_initPI(&VPI, &VPI_SPS, &VPI_CSS,
                       gControlVoltageLoopTsSec,
                       gControlVpiKp, gControlVpiKi,
                       gControlVpiUmax, gControlVpiUmin,
                       gControlVpiImax, gControlVpiImin);

    ControlLoop_initPI(&IPI, &IPI_SPS, &IPI_CSS,
                       gControlCurrentLoopTsSec,
                       gControlIpiKp, gControlIpiKi,
                       gControlIpiUmax, gControlIpiUmin,
                       gControlIpiImax, gControlIpiImin);

    DABCtrl.TargetVoltage = TempTargetVoltage;
    DABCtrl.TargetCurrent = TempTargetCurrent;
    DABCtrl.VoltageLoopOut = 0.0f;
    DABCtrl.CurrentLoopOut = 0.0f;
    DABCtrl.CSS = &DABCSS;
    DABCtrl.SoftStar = &DABSoftStart;

    DABCSS.TargetVoltage = TempTargetVoltage;
    DABCSS.TargetCurrent = TempTargetCurrent;
    DABCSS.sts = DABREADYRUN;
    DABCSS.error = 0U;
    DABCSS.CtrlMode = SPSDAB;
    DABCSS.CtrlLoopTransmissionMode = P2S;
    DABCSS.LoopMode = VMode;

    DABSoftStart.SoftTime = 2.0f;
    ControlLoop_configSoftStart(&DABCtrl);
}


/*==============================================================================
 * 6. 调制输出函数
 *============================================================================*/

/**
 * @brief SPS 单移相输出。
 *
 * 输入：
 *   OutDeg: 原副边外移相角度，单位 deg。
 *
 * 输出：
 *   限幅后调用 Board_EPWM_setAllPhaseDeg() 更新四路 PWM 相位。
 */
void SPSCTRL(float32_t OutDeg)
{
    if (OutDeg>SPS_PHASE_MAX_DEG)  
        OutDeg = SPS_PHASE_MAX_DEG;
    if (OutDeg < -SPS_PHASE_MAX_DEG) {
        OutDeg = -SPS_PHASE_MAX_DEG;
    }
    
    Board_EPWM_setAllPhaseDeg(0.0f,180.0f,0.0f+OutDeg,180+OutDeg);
}

/**
 * @brief EPS 扩展移相输出。
 *
 * 输入：
 *   OutDeg: 闭环输出的外移相角度，单位 deg。
 *
 * 输出：
 *   根据电压比 m 分配原边/副边内移相，并更新 PWM 相位。
 */
void EPSCTRL(float32_t OutDeg)
{   float32_t m =EPS_CONSTANT(DABCtrl.TargetVoltage,gAdcDActual[1],TurnsProportion);

    float32_t PInnerDeg = 0.0f;
    float32_t SInnerDeg = 0.0f;

    if (OutDeg>EPS_PHASE_MAX_DEG)  
        OutDeg = EPS_PHASE_MAX_DEG;
    if (OutDeg < -EPS_PHASE_MAX_DEG) 
        OutDeg = -EPS_PHASE_MAX_DEG;
    if (m<=1) {
        PInnerDeg = 90*(1-m);
        SInnerDeg = 0;
    }
    else 
    {
        PInnerDeg = 0;
        SInnerDeg = 90*(1-(1/m));
    }

    OutDeg = OutDeg - 0.5f * PInnerDeg + 0.5f * SInnerDeg;
    Board_EPWM_setAllPhaseDeg(0.0f,180.0f-PInnerDeg,0.0f+OutDeg,180+OutDeg-SInnerDeg);
}


/* BDPS 双重双向调制优化控制
 *
 * 输入：
 *   D2_in: 外环 PI 给出的外移相占空比命令，理论范围 0~0.5。
 *
 * 输出：
 *   更新 gBdpsLast，并把 D1/D2 换算成 PWM 相位写入 ePWM。
 *
 * D2_in 直接来自电压外环 PI（与原 SPS/EPS 不同：BDPS 是单电压闭环，
 * 不再串联电流内环）。算法内部按 K、p0 选择优化轨迹反解 D1。
 *
 * 注意：当前工程电压 PI 默认 Umax=10.0，如果直接接到 BDPS，会被
 *       BDPS_D2_MAX=0.5 限到饱和。切到 BDPS_OPT 之前需要把
 *       gControlVpiUmax 重新整定到 0.5 量级，否则一直处于饱和。
 *
 * 相位映射沿用 EPS 同一套绝对相位约定（PRI_LEG_A=0 度为同步主基准）：
 *   PriA = 0
 *   PriB = 180 - PInnerDeg
 *   SecA = OutDeg
 *   SecB = 180 + OutDeg - SInnerDeg
 * BDPS 在两侧使用相同的内移相 D1，所以 PInnerDeg = SInnerDeg = D1*180。
 */
void BDPSOPTCTRL(float32_t D2_in)
{
    float32_t PInnerDeg;
    float32_t SInnerDeg;
    float32_t OutDeg;

    /* 优化算法 */
    BDPS_CalcD1_Optimized(gAdcDActual[1],   /* U1 = 原边电压 (ADCD 1) */
                          gAdcCActual[2],   /* U2 = 副边电压 (ADCC 2) */
                          gAdcCActual[0],   /* I2 = 副边电流 (ADCC 0) */
                          D2_in,
                          &gBdpsLast);

    /* 半周期归一化占空比 -> 角度
     * D = 1 对应 180 度（半个开关周期）
     */
    PInnerDeg = gBdpsLast.D1 * 180.0f;
    SInnerDeg = gBdpsLast.D1 * 180.0f;
    OutDeg    = gBdpsLast.D2 * 180.0f;

    Board_EPWM_setAllPhaseDeg(0.0f,
                              180.0f - PInnerDeg,
                              0.0f + OutDeg,
                              180.0f + OutDeg - SInnerDeg);
}


/*==============================================================================
 * 7. 软启动、主控制和 ADC ISR
 *============================================================================*/

/**
 * @brief DAB 软启动控制。
 *
 * 输入：
 *   C: 控制对象，读取目标值、方向、闭环模式和软启动参数。
 *
 * 输出：
 *   更新 SoftTemp、SoftOutput、VoltageLoopOut、CurrentLoopOut，并用 SPS 输出相角。
 */
void DABSoftStar(CtrlLoop *C)
{
    float32_t softRef;

    if ((C == 0) || (C->CSS == 0) || (C->SoftStar == 0))
    {
        return;
    }

    if (C->CSS->sts != DABREADYRUN)
    {
        return;
    }

    if (C->SoftStar->SoftSTS == SoftOver)
    {
        return;
    }

    /* SoftTemp 只做正向幅值爬坡，S2P 的负号在控制器参考值处体现。 */
    if (C->SoftStar->SoftTemp >= C->SoftStar->SoftTarget)
    {
        C->SoftStar->SoftTemp = C->SoftStar->SoftTarget;
        C->SoftStar->SoftSTS = SoftOver;
        ControlLoop_resetPIByMode(C->CSS->LoopMode);
        return;
    }

    C->SoftStar->SoftSTS = SoftRunning;
    C->SoftStar->SoftTemp += C->SoftStar->SoftStep;
    if (C->SoftStar->SoftTemp > C->SoftStar->SoftTarget)
    {
        C->SoftStar->SoftTemp = C->SoftStar->SoftTarget;
    }
    softRef = C->SoftStar->SoftTemp;

    /* 软启动阶段固定用 SPS 缓慢释放相角，运行阶段再切到指定调制。 */
    if (C->CSS->CtrlLoopTransmissionMode == P2S)
    {
        if (C->CSS->LoopMode == VMode)
        {
            C->SoftStar->SoftOutput = ControlLoop_runVoltagePI(C, softRef,
                                                               gAdcCActual[2]);
        }
        else if (C->CSS->LoopMode == IMode)
        {
            C->SoftStar->SoftOutput = DCL_runPI_C3(&IPI, softRef, gAdcCActual[0]);
        }
        else if (C->CSS->LoopMode == VIMode)
        {
            ControlLoop_runVoltagePI(C, softRef, gAdcCActual[2]);
            C->SoftStar->SoftOutput = DCL_runPI_C3(&IPI, C->VoltageLoopOut,
                                                   gAdcCActual[0]);
        }
        else
        {
            C->SoftStar->SoftSTS = SoftError;
            C->CSS->sts = DABERROR;
            return;
        }

        SPSCTRL(C->SoftStar->SoftOutput);
    }
    else if (C->CSS->CtrlLoopTransmissionMode == S2P)
    {
        if (C->CSS->LoopMode == VMode)
        {
            C->SoftStar->SoftOutput = ControlLoop_runVoltagePI(C, softRef,
                                                               gAdcDActual[1]);
            C->SoftStar->SoftOutput = -C->SoftStar->SoftOutput;
        }
        else if (C->CSS->LoopMode == IMode)
        {
            C->SoftStar->SoftOutput = DCL_runPI_C3(&IPI, -softRef, gAdcCActual[0]);
        }
        else if (C->CSS->LoopMode == VIMode)
        {
            ControlLoop_runVoltagePI(C, softRef, gAdcDActual[1]);
            C->SoftStar->SoftOutput = DCL_runPI_C3(&IPI, -C->VoltageLoopOut,
                                                   gAdcCActual[0]);
        }
        else
        {
            C->SoftStar->SoftSTS = SoftError;
            C->CSS->sts = DABERROR;
            return;
        }

        SPSCTRL(C->SoftStar->SoftOutput);
    }
    else
    {
        C->SoftStar->SoftSTS = SoftError;
        C->CSS->sts = DABERROR;
    }
}

/**
 * @brief 把闭环输出分发到具体调制算法。
 *
 * 输入：
 *   ctrlMode: SPSDAB/EPSDAB/TPSDAB/BDPSOPTDAB。
 *   out: 闭环输出，单位随调制算法而定。SPS/EPS 为 deg，BDPS 为 D2。
 *
 * 输出：
 *   调用对应调制函数写 PWM。非法模式或未实现 TPS 会退回 SPSCTRL(0.0f)。
 */
static void ControlLoop_applyOutput(uint32_t ctrlMode, float32_t out)
{
    /* 根据调制模式把控制器输出映射到实际 PWM 相位。 */
    switch (ctrlMode)
    {
        case SPSDAB:
            SPSCTRL(out);
            break;

        case EPSDAB:
            EPSCTRL(out);
            break;

        case BDPSOPTDAB:
            BDPSOPTCTRL(out);
            break;

        case TPSDAB:
        default:
            /*
             * TPS 尚未实现或 CtrlMode 非法时不能空操作，否则 PWM 会保持上一拍相角。
             * 这里先退回 0 外移相，相当于不主动传输功率。
             */
            SPSCTRL(0.0f);
            break;
    }
}

/**
 * @brief 软启动完成后的正常闭环主控制。
 *
 * 输入：
 *   C: 控制对象，读取目标值、方向、闭环模式和调制模式。
 *
 * 输出：
 *   更新 VoltageLoopOut/CurrentLoopOut，并通过调制函数写 PWM 相角。
 */
static void ControlLoop_runMainCtrl(CtrlLoop *C)
{
    if ((C == 0) || (C->CSS == 0))
    {
        return;
    }

    if (C->CSS->CtrlLoopTransmissionMode == P2S)
    {
        /* P2S：以副边电压/副边电流为闭环反馈，正相角传能到副边。 */
        if (C->CSS->LoopMode == VMode)
        {
            ControlLoop_runVoltagePI(C, C->TargetVoltage, gAdcCActual[2]);
            ControlLoop_applyOutput(C->CSS->CtrlMode, C->VoltageLoopOut);
        }
        else if (C->CSS->LoopMode == IMode)
        {
            C->CurrentLoopOut = DCL_runPI_C3(&IPI, C->TargetCurrent, gAdcCActual[0]);
            ControlLoop_applyOutput(C->CSS->CtrlMode, C->CurrentLoopOut);
        }
        else if (C->CSS->LoopMode == VIMode)
        {
            ControlLoop_runVoltagePI(C, C->TargetVoltage, gAdcCActual[2]);
            C->CurrentLoopOut = DCL_runPI_C3(&IPI, C->VoltageLoopOut, gAdcCActual[0]);
            ControlLoop_applyOutput(C->CSS->CtrlMode, C->CurrentLoopOut);
        }
        else
        {
            C->CSS->error = 1U;
            C->CSS->sts = DABERROR;
        }
    }
    else if (C->CSS->CtrlLoopTransmissionMode == S2P)
    {
        /* S2P：以原边电压为电压反馈，电流参考或输出相角取反实现反向传能。 */
        if (C->CSS->LoopMode == VMode)
        {
            ControlLoop_runVoltagePI(C, C->TargetVoltage, gAdcDActual[1]);
            ControlLoop_applyOutput(C->CSS->CtrlMode, -C->VoltageLoopOut);
        }
        else if (C->CSS->LoopMode == IMode)
        {
            C->CurrentLoopOut = DCL_runPI_C3(&IPI, -C->TargetCurrent, gAdcCActual[0]);
            ControlLoop_applyOutput(C->CSS->CtrlMode, C->CurrentLoopOut);
        }
        else if (C->CSS->LoopMode == VIMode)
        {
            ControlLoop_runVoltagePI(C, C->TargetVoltage, gAdcDActual[1]);
            C->CurrentLoopOut = DCL_runPI_C3(&IPI, -C->VoltageLoopOut, gAdcCActual[0]);
            ControlLoop_applyOutput(C->CSS->CtrlMode, C->CurrentLoopOut);
        }
        else
        {
            C->CSS->error = 1U;
            C->CSS->sts = DABERROR;
        }
    }
    else
    {
        C->CSS->error = 1U;
        C->CSS->sts = DABERROR;
    }
}

/**
 * @brief ADC ISR 中调用的控制主入口。
 *
 * 输入：
 *   ADC 归一化采样值来自 board_adc.c。
 *   TempTargetVoltage / TempTargetCurrent 可由外部调试或上位机修改。
 *
 * 输出：
 *   换算 ADC 实际值、同步目标值、更新 PI 参数、执行软启动或主控制。
 */
void ControlLoop_adcISR(void)
{

    /*ADC数据处理放这里*/

    Board_ADC_ConvertNormToActual();

    /*ADC数据处理完*/

    /*此处进行过流保护*/

    if (ControlLoop_checkOverload())
    {
        return;
    }
    /*过流检测通过*/


    /*目标值和PID参数更新放这里*/

    if(DABCSS.sts == DABWAITCHANGE)  
    {
        uint32_t nextSts = DABRUNING;

        /*
         * 运行过程中只修改目标电压/目标电流时，不软启动。
         */
        DABCtrl.TargetVoltage = TempTargetVoltage;
        DABCtrl.TargetCurrent = TempTargetCurrent;
        DABCSS.TargetVoltage = TempTargetVoltage;
        DABCSS.TargetCurrent = TempTargetCurrent;
        DABCSS.error = 0U;

        /*
         * 如果目标是在上电软启动尚未完成时被修改，则继续保持软启动状态；
         * 如果软启动已经完成，则直接回到正常运行状态，不再从 0 爬坡。
         */
        if ((DABCtrl.SoftStar != 0) && (DABCtrl.SoftStar->SoftSTS != SoftOver))
        {
            nextSts = DABREADYRUN;
        }

        DABCSS.sts = nextSts;
    }

        if (DCL_UPDATE_WAITING(&VPI))
    {
        DCL_updatePI(&VPI);
    }

            if (DCL_UPDATE_WAITING(&IPI))
    {
        DCL_updatePI(&IPI);
    }
    /*更新完成*/

    /*缓启动放这里，默认使用SPS缓启动*/

    DABSoftStar(&DABCtrl);

    if (DABCtrl.CSS->sts == DABREADYRUN && DABCtrl.SoftStar->SoftSTS == SoftOver)
    {
        /* 软启动结束后进入正常运行，后续由主控制代码接管 PWM 相角。 */
        DABCtrl.CSS->sts = DABRUNING;
    }
    
    /*缓启动完成*/


    //按照模式控制
    //控制主代码

    if (DABCtrl.CSS->sts == DABRUNING ) 
    {
        ControlLoop_runMainCtrl(&DABCtrl);
    }
    
    //控制完成，并且更新参数了退出中断
    

    
    // ControlLoop_requestVoltagePIUpdate();
    // ControlLoop_requestCurrentPIUpdate();
    /*如果有参数要更改的话请务必调用这个函数，不要直接修改寄存器的值，这个写在处理上位机的程序里面不要放中断函数
    修改方法eg：上位机通过指针改全局变量的电压环PI的P值，
    然后调用一次ControlLoop_requestVoltagePIUpdate，就可以了，
    收到请求更新之后下次中断开始会自己更新的，但是别放进while里面，每次都更新一次值影响运行效率
    */
}

/**
 * @brief 主循环中的慢任务入口。
 *
 * 输入：无。
 * 输出：当前为空，后续可放低频通信、参数保存、状态上报等任务。
 */
void ControlLoop_slowTask(void)//放mainwhile
{
}
