#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#define SPS_PHASE_MAX_DEG   30.0f
#define EPS_PHASE_MAX_DEG   30.0f
#define EPS_CONSTANT(a,b,c)   ((float32_t)(a*c/b))
#define TPS_CONSTANT(a,b,c)   ((float32_t)(a*c/b))

#include "board.h"
#define NULL_ADDR   0x00000000

enum CtrlLoopTransmissionMode{
    P2S=0,
    S2P
} ;

enum CtrlLoopError{
    
    PrimarySideTrankCurrentOverLoard=0,
    SecondSideTrankCurrentOverLoard,
    PrimarySideInputCurrentOverLoard,
    SecondSideInputCurrentOverLoard,
    PrimarySideInputVoltageOverLoard,
    SecondSideInputVoltageOverLoard

} ;

enum CtrlMode{
    SPSDAB=0,
    EPSDAB,
    TPSDAB,
    BDPSOPTDAB        /* BDPS 双重双向调制优化控制 */
} ;

enum DABSTS{
    DABREADYRUN=0,
    DABRUNING,
    DABWAITCHANGE,
    DABERROR    
} ;

/*
 *VMode 单电压环
 *IMode 单电流环
 *VIMode 电压电流环
*/
enum LoopMode
{
    VMode = 0,
    IMode,
    VIMode
};


typedef struct{
    
    float32_t TargetVoltage;
    float32_t TargetCurrent;

    uint32_t sts;
    uint32_t error;
    uint32_t CtrlMode;
    uint32_t CtrlLoopTransmissionMode;
    uint32_t LoopMode;
    
} CTRLCSS;

enum SoftStaut{
    SoftRunning =0,
    SoftOver,
    SoftError
};


typedef struct
{
    uint16_t SoftSTS;

    float32_t SoftStep;
    float32_t SoftTarget;
    float32_t SoftTime;
    float32_t SoftTemp;

    float32_t SoftOutput;

} DAB_SoftStart_t;


typedef struct{

    float32_t TargetVoltage;
    float32_t TargetCurrent;

    float32_t VoltageLoopOut;
    float32_t CurrentLoopOut;

    CTRLCSS *CSS;
    DAB_SoftStart_t *SoftStar;


}CtrlLoop;



#define CTRLCSSDefaults {0.0f,0.0f,DABREADYRUN,0U,SPSDAB,P2S,VMode}
#define CtrlLoopDefaults {0.0f,0.0f,0.0f,0.0f,NULL_ADDR,NULL_ADDR}

void ControlLoop_init(void);
void ControlLoop_adcISR(void);
void ControlLoop_slowTask(void);
void ControlLoop_requestVoltagePIUpdate(void);
void ControlLoop_requestCurrentPIUpdate(void);
void ControlLoop_requestAllPIUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOOP_H */
