#include "dj_motor.h"
#include "fdcan.h"       /* hfdcan2：发送 CAN 帧用 */

#define ABS(x)          (((x) > 0) ? (x) : -(x))
#define CLAMP(x, lo, hi) (((x) < (lo)) ? (lo) : ((x) > (hi)) ? (hi) : (x))
#define CLAMPPEAK(x, pk) (((x) > (pk)) ? (pk) : ((x) < -(pk)) ? -(pk) : (x))
#define GETSIGN(x)      (((x) >= 0) ? 1 : -1)

DJMotor DJmotor[1];//电机数量为1

void DJmotor_Init(void)
{
    /* ---- M2006参数 ---- */
    djmotor_param param;
    param.Gear_ratio       = 1.0f;         
    param.Reduction_ratio  = M2006_RATIO;    
    param.PulsePerRound    = M2006_PULSE_RD;   
    param.CurrentLimit_raw = M2006_CUR_LIMIT;  

    /* ---- 限幅 ---- */
    djmotor_limit limit;
    limit.RPMLimitFlag        = false;
    limit.SpeedRPMLimit       = 10000;
    limit.PosAngleLimitFlag   = false;
    limit.MaxAngle_deg        = 270.0f;
    limit.MinAngle_deg        = -270.0f;
    limit.PosRPMFlag          = true;
    limit.PosRPMLimit         = 8000;
    limit.CurrentLimitFlag    = true;
    limit.ZeroRPMLimit        = 500;
    limit.ZeroCurrentLimit_raw = 3000;
    limit.IsLooseStuck        = false;

    /* ---- 状态 ---- */
    djmotorstatus_flag status;
    status.IsSetZero    = true;     /* 第一帧反馈到就把当前位置清零 */
    status.OvertimeFlag = false;
    status.StuckFlag    = false;
    status.ZeroFlag     = false;

    /* ---- 内部计数 ---- */
    djmotor_argum argum;
    argum.zeroCnt   = 0;
    argum.GapCnt    = 0;
    argum.pulseLock = 0;

    /* ---- 监测计数 ---- */
    djmotor_error error;
    error.LastRxTime   = 0;
    error.stuckCount   = 0;
    error.timeoutCount = 0;

    /* ---- 应用到所有电机 ---- */
    for (uint32_t i = 0; i < USE_DJNUM; i++)
    {
        DJmotor[i].ID         = (uint8_t)(i + 1U);   
        DJmotor[i].Begin      = false;            
        DJmotor[i].MODE_Set   = DJ_Disable;         
        DJmotor[i].MODE_Cur   = DJ_Disable;
        DJmotor[i].param      = param;
        DJmotor[i].limit      = limit;
        DJmotor[i].statusFlag = status;
        DJmotor[i].argum      = argum;
        DJmotor[i].error      = error;

        /* PID 设立初值 */
        pid_init(&DJmotor[i].posPID, 0.07f, 0.005f, 0.0f, PIDPOS);
        pid_init(&DJmotor[i].velPID, 5.5f, 0.3f, 0.01f, PIDINC);
    }
}

