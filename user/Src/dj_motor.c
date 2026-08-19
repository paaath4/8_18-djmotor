#include "dj_motor.h"
#include "fdcan.h"       /* hfdcan2：发送 CAN 帧用 */

#define ABS(x)          (((x) > 0) ? (x) : -(x))
#define CLAMP(x, lo, hi) (((x) < (lo)) ? (lo) : ((x) > (hi)) ? (hi) : (x))
#define CLAMPPEAK(x, pk) (((x) > (pk)) ? (pk) : ((x) < -(pk)) ? -(pk) : (x))
#define GETSIGN(x)      (((x) >= 0) ? 1 : -1)
#define ZERO_DISTANCE   5  /* 脉冲差 < 5 视为撞到限位 */

DJMotor DJmotor[4];/*电机数量4*/

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

/* 置零 */
static void DJmotor_SetZero(DJMotorPointer motor)
{
    motor->valNow.PulseTotal = 0;
    motor->valNow.angle_deg  = 0.0f;
    motor->valNow.PulseTotal = 0;
    motor->argum.pulseLock   = 0;
}

/* 计算转子里程 */
static void DJmotor_AngleCalculate(DJMotorPointer motor)
{
    /* 相邻帧脉冲差 */
    motor->valNow.PulseGap = (int16_t)(motor->valNow.PulseRead - motor->valPre.PulseRead);

    /* 回绕 */
    if (ABS(motor->valNow.PulseGap) > (int16_t)(motor->param.PulsePerRound / 2))
    {
        motor->valNow.PulseGap = (int16_t)(motor->valNow.PulseGap
                - GETSIGN(motor->valNow.PulseGap) * ((int32_t)motor->param.PulsePerRound + 1));
    }

    /*累加里程 */
    motor->valNow.PulseTotal += motor->valNow.PulseGap;

    /*计算 */
    motor->valNow.angle_deg = (float)motor->valNow.PulseTotal * 360.0f
            / ((float)motor->param.PulsePerRound
             * motor->param.Gear_ratio
             * motor->param.Reduction_ratio);

    /* 置零 */
    if (motor->statusFlag.IsSetZero)
    {
        DJmotor_SetZero(motor);
        motor->statusFlag.IsSetZero = false;
    }

    /* 保存上一帧，供下次差分 */
    motor->valPre = motor->valNow;
}

/*传输*/
static void DJmotor_CurrentTransmit(DJMotorPointer motor)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8] = {0};

    tx_header.IdType            = FDCAN_STANDARD_ID;
    tx_header.TxFrameType       = FDCAN_DATA_FRAME;
    tx_header.DataLength        = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch     = FDCAN_BRS_OFF;
    tx_header.FDFormat          = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl= FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker     = 0;

    if (motor->ID <= 4U)
    {
        tx_header.Identifier = 0x200U;
        tx_data[(motor->ID - 1U) * 2U]      = (uint8_t)(motor->valSet.current_raw >> 8); /* 高字节 */
        tx_data[(motor->ID - 1U) * 2U + 1U] = (uint8_t)(motor->valSet.current_raw);       /* 低字节 */
    }
    else
    {
        tx_header.Identifier = 0x1FFU;
        tx_data[(motor->ID - 5U) * 2U]      = (uint8_t)(motor->valSet.current_raw >> 8);
        tx_data[(motor->ID - 5U) * 2U + 1U] = (uint8_t)(motor->valSet.current_raw);
    }

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_data);
}



/*接收*/
void DJmotor_Receive(FDCAN_RxHeaderTypeDef RxHeader, uint8_t *Rx_data)
{
    /*收标准数据帧 */
    if ((RxHeader.IdType      != FDCAN_STANDARD_ID) ||
        (RxHeader.RxFrameType != FDCAN_DATA_FRAME)  ||
        (RxHeader.Identifier  <  0x201U)            ||
        (RxHeader.Identifier  >  0x208U))
    {
        return;
    }

    uint8_t card_id = (uint8_t)(RxHeader.Identifier - 0x200U);   
    if (card_id > USE_DJNUM) 
    {
        return;
    }

    DJMotorPointer motor = &DJmotor[card_id - 1U];   

    motor->valNow.PulseRead     = (int16_t)(((uint16_t)Rx_data[0] << 8) | Rx_data[1]);
    motor->valNow.speed_rpm     = (int16_t)(((uint16_t)Rx_data[2] << 8) | Rx_data[3]);
    motor->valNow.current_raw   = (int16_t)(((uint16_t)Rx_data[4] << 8) | Rx_data[5]);
    motor->valNow.temperature_C = (int8_t)Rx_data[6];

    motor->valNow.current_A = (float)motor->valNow.current_raw / 10000.0f * 10.0f;

    motor->valNow.speed_rpm = (int16_t)((float)motor->valNow.speed_rpm
            / (motor->param.Gear_ratio * motor->param.Reduction_ratio));

    motor->error.LastRxTime = 0;

    DJmotor_AngleCalculate(motor);
}






/* 模式切换 */
static void DJmotor_SwitchMode(DJMotorPointer motor)
{
    if (motor->MODE_Set != motor->MODE_Cur)
    {
        motor->MODE_Cur = motor->MODE_Set;


        motor->valSet.current_raw = 0;
        motor->valSet.speed_rpm   = 0;


        motor->valSet.angle_deg = motor->valNow.angle_deg;

        pid_reset(&motor->posPID);
        pid_reset(&motor->velPID);

        motor->statusFlag.ZeroFlag     = false;
        motor->statusFlag.OvertimeFlag = false;
        motor->statusFlag.StuckFlag    = false;
    }
}



/* 速度模式*/
static void DJmotor_SpeedMode(DJMotorPointer motor)
{
    motor->velPID.setval = (float)motor->valSet.speed_rpm
            * motor->param.Gear_ratio * motor->param.Reduction_ratio;
    motor->velPID.curval = (float)motor->valNow.speed_rpm
            * motor->param.Gear_ratio * motor->param.Reduction_ratio;

    /* 速度限幅 */
    if (motor->limit.RPMLimitFlag)
    {
        motor->velPID.setval = CLAMPPEAK(motor->velPID.setval, motor->limit.SpeedRPMLimit);
    }

    motor->valSet.current_raw += (int16_t)pid_calculate(&motor->velPID);

    /* 电流限幅 */
    motor->valSet.current_raw = (int16_t)CLAMPPEAK(motor->valSet.current_raw,
            motor->param.CurrentLimit_raw);
}


/* 位置模式 */
static void DJmotor_PositionMode(DJMotorPointer motor)
{
    motor->posPID.setval = motor->valSet.angle_deg * motor->param.Gear_ratio
            * motor->param.Reduction_ratio * (float)motor->param.PulsePerRound / 360.0f;

    /* 位置限幅 */
    if (motor->limit.PosAngleLimitFlag)
    {
        float max_pulse = motor->limit.MaxAngle_deg * motor->param.Gear_ratio
                * motor->param.Reduction_ratio * (float)motor->param.PulsePerRound / 360.0f;
        float min_pulse = motor->limit.MinAngle_deg * motor->param.Gear_ratio
                * motor->param.Reduction_ratio * (float)motor->param.PulsePerRound / 360.0f;
        motor->posPID.setval = CLAMP(motor->posPID.setval, min_pulse, max_pulse);
    }

    /* 位置环：位置误差 → 速度指令（位置式 PID） */
    motor->posPID.curval = (float)motor->valNow.PulseTotal;
    motor->velPID.setval = pid_calculate(&motor->posPID);

    motor->velPID.curval = (float)motor->valNow.speed_rpm
            * motor->param.Gear_ratio * motor->param.Reduction_ratio;

    /* 位置环内速度限幅 */
    if (motor->limit.PosRPMFlag)
    {
        motor->velPID.setval = CLAMPPEAK(motor->velPID.setval, motor->limit.PosRPMLimit);
    }

    /* 速度环：增量式 PID，输出 Δ 累加电流 */
    motor->valSet.current_raw += (int16_t)pid_calculate(&motor->velPID);

    /* 电流限幅 */
    motor->valSet.current_raw = (int16_t)CLAMPPEAK(motor->valSet.current_raw,
            motor->param.CurrentLimit_raw);
}


/* 寻零模式*/
static void DJmotor_ZeroMode(DJMotorPointer motor)
{
    /* 恒速驱动 */
    motor->velPID.setval = (float)motor->limit.ZeroRPMLimit;
    motor->velPID.curval = (float)motor->valNow.speed_rpm;

    motor->valSet.current_raw += (int16_t)pid_calculate(&motor->velPID);
    motor->valSet.current_raw = (int16_t)CLAMPPEAK(motor->valSet.current_raw,
            motor->limit.ZeroCurrentLimit_raw);

    /* 停滞判定：脉冲差小于阈值，连续 100 帧（100ms）认为撞到限位 */
    if (ABS(motor->valNow.PulseGap) < ZERO_DISTANCE)
    {
        motor->argum.zeroCnt++;
        if (motor->argum.zeroCnt > 100U)
        {
            motor->argum.zeroCnt = 0;
            motor->statusFlag.ZeroFlag = true;
            motor->Begin = false;              
            pid_reset(&motor->posPID);
            pid_reset(&motor->velPID);
            DJmotor_SetZero(motor);           
        }
    }
    else
    {
        motor->argum.zeroCnt = 0;            
    }
}


/* 监测堵转和失联 */
static void DJmotor_Monitor(DJMotorPointer motor)
{
    if ((ABS(motor->valNow.PulseGap) < 5) && (motor->valNow.current_raw > 3000))
    {
        motor->error.stuckCount++;
        if (motor->error.stuckCount > 500U)  
        {
            motor->error.stuckCount = 0;
            motor->statusFlag.StuckFlag = true;
            if (motor->limit.IsLooseStuck)
            {
                motor->MODE_Set = DJ_Disable;   /* 堵转自动断电 */
            }
        }
    }
    else
    {
        motor->error.stuckCount = 0;            /* 正常转动则清零 */
    }

    /*失联*/
    motor->error.LastRxTime++;              
    if (motor->error.LastRxTime > 50U)
    {
        motor->error.timeoutCount++;
        if (motor->error.timeoutCount > 20U)
        {
            motor->error.timeoutCount = 0;
            motor->statusFlag.OvertimeFlag = true;
            motor->MODE_Set = DJ_Disable;       /* 失联断电 */
        }
    }
    else
    {
        motor->error.timeoutCount = 0;          /* 通信恢复则清零 */
    }
}

/*状态机*/
void DJmotor_Func(void)
{
    for (uint32_t i = 0; i < USE_DJNUM; i++)
    {
        if (DJmotor[i].Begin)
        {
            DJmotor_Monitor(&DJmotor[i]);               
            DJmotor_SwitchMode(&DJmotor[i]);            

            switch (DJmotor[i].MODE_Cur)
            {
                case DJ_Disable:
                    DJmotor[i].valSet.current_raw = 0;
                    break;
                case DJ_RPM:
                    DJmotor_SpeedMode(&DJmotor[i]);
                    break;
                case DJ_Position:
                    DJmotor_PositionMode(&DJmotor[i]);
                    break;
                case DJ_Zero:
                    DJmotor_ZeroMode(&DJmotor[i]);
                    break;
                case DJ_Current:
                    DJmotor[i].valSet.current_raw =
                        (int16_t)CLAMPPEAK(DJmotor[i].valSet.current_raw,
                                           DJmotor[i].param.CurrentLimit_raw);
                    break;
                default:
                    break;
            }
        }
        else
        {
            DJmotor[i].valSet.current_raw = 0;
        }

        DJmotor_CurrentTransmit(&DJmotor[i]);
    }
}

static void DJmotor_PID_Reload(DJMotorPointer motor, uint8_t which, float kp, float ki, float kd)
{
    if (which == 0U)
    {
        motor->posPID.kp = kp;
        motor->posPID.ki = ki;
        motor->posPID.kd = kd;
    }
    else
    {
        motor->velPID.kp = kp;
        motor->velPID.ki = ki;
        motor->velPID.kd = kd;
    }
}

