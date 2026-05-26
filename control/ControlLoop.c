#include "ControlLoop.h"
#include "board.h"
#include "DCL.h"
#include "DCLF32.h"
#include "dab_bdps_opt.h"

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
volatile float32_t gControlVoltageLoopTsSec = 1.0e-6f;

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


CtrlLoop DABCtrl =  CtrlLoopDefaults;
CTRLCSS  DABCSS = CTRLCSSDefaults;
DAB_SoftStart_t  DABSoftStart;

DCL_PI VPI = PI_DEFAULTS;    //这里初始化PI结构体，DCL的结构体里面自己有一个默认的初始化值
DCL_PI_SPS VPI_SPS = PI_SPS_DEFAULTS;//影子结构体,DCL的结构体里面自己有一个默认的初始化值
DCL_CSS VPI_CSS = DCL_CSS_DEFAULTS;//默认的结构体，处理报错等信息,DCL的结构体里面自己有一个默认的初始化值
//下面同理上面
DCL_PI IPI = PI_DEFAULTS;
DCL_PI_SPS IPI_SPS = PI_SPS_DEFAULTS;
DCL_CSS IPI_CSS = DCL_CSS_DEFAULTS;


//初始化函数

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

void ControlLoop_requestVoltagePIUpdate(void)//更新电压环
{
    ControlLoop_requestPIUpdate(&VPI, &VPI_SPS,
                                gControlVpiKp, gControlVpiKi,
                                gControlVpiUmax, gControlVpiUmin,
                                gControlVpiImax, gControlVpiImin);
}

void ControlLoop_requestCurrentPIUpdate(void)//更新电流环
{
    ControlLoop_requestPIUpdate(&IPI, &IPI_SPS,
                                gControlIpiKp, gControlIpiKi,
                                gControlIpiUmax, gControlIpiUmin,
                                gControlIpiImax, gControlIpiImin);
}

void ControlLoop_requestAllPIUpdate(void)
{
    ControlLoop_requestVoltagePIUpdate();
    ControlLoop_requestCurrentPIUpdate();
}

static float32_t ControlLoop_absF32(float32_t x)
{
    return (x < 0.0f) ? -x : x;
}

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
}

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





void ControlLoop_init(void)//中断初始化
{
    gControlVoltageLoopTsSec = gControlCurrentLoopTsSec *
                               (float32_t)gControlVoltageLoopDiv;

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
    
   
    {
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

    


}





void SPSCTRL(float32_t OutDeg)
{
    if (OutDeg>SPS_PHASE_MAX_DEG)  
        OutDeg = SPS_PHASE_MAX_DEG;
    if (OutDeg < -SPS_PHASE_MAX_DEG) {
        OutDeg = -SPS_PHASE_MAX_DEG;
    }
    
    Board_EPWM_setAllPhaseDeg(0.0f,180.0f,0.0f+OutDeg,180+OutDeg);
}

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

    /* 软启动阶段固定用 SPS 缓慢释放相角，运行阶段再切到 CtrlMode 指定调制。 */
    if (C->CSS->CtrlLoopTransmissionMode == P2S)
    {
        if (C->CSS->LoopMode == VMode)
        {
            C->SoftStar->SoftOutput = DCL_runPI_C3(&VPI, softRef, gAdcCActual[2]);
        }
        else if (C->CSS->LoopMode == IMode)
        {
            C->SoftStar->SoftOutput = DCL_runPI_C3(&IPI, softRef, gAdcCActual[0]);
        }
        else if (C->CSS->LoopMode == VIMode)
        {
            C->VoltageLoopOut = DCL_runPI_C3(&VPI, softRef, gAdcCActual[2]);
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
            C->SoftStar->SoftOutput = DCL_runPI_C3(&VPI, softRef, gAdcDActual[1]);
            C->SoftStar->SoftOutput = -C->SoftStar->SoftOutput;
        }
        else if (C->CSS->LoopMode == IMode)
        {
            C->SoftStar->SoftOutput = DCL_runPI_C3(&IPI, -softRef, gAdcCActual[0]);
        }
        else if (C->CSS->LoopMode == VIMode)
        {
            C->VoltageLoopOut = DCL_runPI_C3(&VPI, softRef, gAdcDActual[1]);
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
            break;
    }
}

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
            C->VoltageLoopOut = DCL_runPI_C3(&VPI, C->TargetVoltage, gAdcCActual[2]);
            ControlLoop_applyOutput(C->CSS->CtrlMode, C->VoltageLoopOut);
        }
        else if (C->CSS->LoopMode == IMode)
        {
            C->CurrentLoopOut = DCL_runPI_C3(&IPI, C->TargetCurrent, gAdcCActual[0]);
            ControlLoop_applyOutput(C->CSS->CtrlMode, C->CurrentLoopOut);
        }
        else if (C->CSS->LoopMode == VIMode)
        {
            C->VoltageLoopOut = DCL_runPI_C3(&VPI, C->TargetVoltage, gAdcCActual[2]);
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
            C->VoltageLoopOut = DCL_runPI_C3(&VPI, C->TargetVoltage, gAdcDActual[1]);
            ControlLoop_applyOutput(C->CSS->CtrlMode, -C->VoltageLoopOut);
        }
        else if (C->CSS->LoopMode == IMode)
        {
            C->CurrentLoopOut = DCL_runPI_C3(&IPI, -C->TargetCurrent, gAdcCActual[0]);
            ControlLoop_applyOutput(C->CSS->CtrlMode, C->CurrentLoopOut);
        }
        else if (C->CSS->LoopMode == VIMode)
        {
            C->VoltageLoopOut = DCL_runPI_C3(&VPI, C->TargetVoltage, gAdcDActual[1]);
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

uint8_t CurrentLoopCount = 0;


extern volatile float gAdcCActual[BOARD_ADC_DIFF_CHANNEL_COUNT] ;
extern volatile float gAdcDActual[BOARD_ADC_DIFF_CHANNEL_COUNT] ;
/*
 *AdcC 0 = Second Side  Current
 *ADCC 1 = Second Side Trank Current
 *ADCC 2 = Second Side  Voltage 
 *ADCD 0 = Primary Side Trank Current 
 *ADCD 1 = Primary Side Voltage
 *ADCD 2 = Primary Side Current
*/


void ControlLoop_adcISR(void)
{

    /*ADC数据处理放这里*/

    Board_ADC_ConvertNormToActual();

    /*ADC数据处理完*/

    /*目标值和PID参数更新放这里*/

    if(DABCSS.sts == DABWAITCHANGE)  
    {
        uint32_t nextSts = DABRUNING;

        /*
         * 运行过程中只修改目标电压/目标电流时，不需要重新软启动。
         * 这里仅把临时目标值同步到控制目标，PI 积分和当前 PWM 相角继续保留，
         * 让闭环自己平滑跟踪新目标。
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
    //控制主代码放这里
    if (DABCtrl.CSS->sts == DABRUNING ) 
    {
        ControlLoop_runMainCtrl(&DABCtrl);
    }
    
    //控制完成，并且更新参数了退出中断
    


   
    
    // ControlLoop_requestVoltagePIUpdate();
    // ControlLoop_requestCurrentPIUpdate();
    //如果有参数要更新的话这个写在处理上位机的程序里面不要放中断函数

}

void ControlLoop_slowTask(void)//放mainwhile
{
}
