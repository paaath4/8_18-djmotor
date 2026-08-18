#ifndef _DJ_MOTOR_H_
#define _DJ_MOTOR_H_

#include "main.h"
#include "stdbool.h"
#include "pid.h"

/* ============ 配置宏 ============ */
#define M2006_RATIO     36.0f    /* M2006 减速比 36:1 */
#define M2006_PULSE_RD  8191U    /* 编码器每圈脉冲（反馈角度 0~8191） */
#define M2006_CUR_LIMIT 4500     /* M2006 默认电流限幅 raw */
#define USE_DJNUM  1U

typedef enum 
{
    DJ_Disable = 0, /*释放模式*/
    DJ_RPM = 1,     /*速度模式*/
    DJ_Position = 2,/*位置模式*/
    DJ_Zero = 3,    /*寻零模式*/
    DJ_Current = 4, /*电流/扭矩*/
} DJmotor_mode_t;    /*电机模式*/

typedef struct
{
    volatile int16_t current_raw;  /* 电流指令/反馈 raw */
    volatile float   angle_deg;    /* 输出轴角度 (°)*/
    volatile int16_t speed_rpm;    /* 输出轴转速 (rpm)*/
    volatile float   current_A;    /* 反馈电流 (A) */
    volatile int16_t PulseRead;    /* 原始编码器脉冲 0~8191*/
    volatile int16_t PulseGap;     /* 相邻两帧脉冲差 */
    volatile int32_t PulseTotal;   /* 累计脉冲（位置里程）*/
    volatile int8_t  temperature_C;/* 电机温度 (°C) */
} DJmotorVal;

typedef struct
{
    float    Gear_ratio;        /* 机构传动比（直连 = 1.0） */
    float    Reduction_ratio;   /* 电机减速比（M2006 = 36） */
    uint16_t PulsePerRound;     /* 编码器每圈脉冲（8191） */
    int16_t  CurrentLimit_raw;  /* 电流指令限幅 raw */
} djmotor_param;

typedef struct
{
    bool     RPMLimitFlag;         /* 速度模式限速使能 */
    bool     PosAngleLimitFlag;    /* 位置角度限幅使能 */
    bool     PosRPMFlag;           /* 位置环内速度限幅使能 */
    bool     CurrentLimitFlag;     /* 电流限幅使能 */
    float    MaxAngle_deg;         /* 位置上限 (°) */
    float    MinAngle_deg;         /* 位置下限 (°) */
    int16_t  SpeedRPMLimit;        /* 速度模式转速上限 (rpm) */
    int32_t  PosRPMLimit;          /* 位置环速度上限 (rpm) */
    int16_t  ZeroRPMLimit;         /* 寻零转速 (rpm) */
    int16_t  ZeroCurrentLimit_raw; /* 寻零电流限幅 raw */
    bool     IsLooseStuck;         /* true：堵转后自动失能 */
} djmotor_limit; /*电机限制*/

typedef struct
{
    bool IsSetZero;     /* 请求当前位置清零 */
    bool OvertimeFlag;  /* 通信超时标志 */
    bool StuckFlag;     /* 堵转标志 */
    bool ZeroFlag;      /* 寻零完成标志 */
} djmotorstatus_flag;

typedef struct
{
    uint32_t zeroCnt;    /* 寻零停滞累计 */
    uint32_t GapCnt;     /* (保留) */
    int32_t  pulseLock;  /* (废弃字段，占位保留) */
} djmotor_argum;

typedef struct
{
    uint16_t LastRxTime;    /* 距上次收包的tick数*/
    uint16_t stuckCount;    /* 堵转连续计数 */
    uint16_t timeoutCount;  /* 超时连续计数 */
} djmotor_error;

/* ============ 电机对象 ============ */
typedef struct
{
    uint8_t                 ID;         /* 电机 ID = 数组下标 + 1 */
    volatile bool           Begin;      /* true：使能运行；false：失能发 0 电流 */
    volatile DJmotor_mode_t MODE_Set;   /* 目标模式（任务层写） */
    volatile DJmotor_mode_t MODE_Cur;   /* 实际运行模式（驱动层维护） */
    djmotor_param       param;
    DJmotorVal  valSet;     /* 期望值，任务层写 */
    DJmotorVal  valNow;     /* 反馈值，驱动层收包写 */
    DJmotorVal  valPre;     /* 上一帧反馈，驱动层内部用 */
    djmotorstatus_flag statusFlag;
    djmotor_limit       limit;
    djmotor_argum       argum;
    djmotor_error       error;
    pidtype posPID;   /* 位置环（位置式） */
    pidtype velPID;   /* 速度环（增量式） */
} DJMotor, *DJMotorPointer;

/* ============ 对外接口 ============ */
extern DJMotor DJmotor[];  

void DJmotor_Init(void);                                 /* 电机初始化：参数/状态/PID 增益 */
void DJmotor_Func(void);                                 /* 状态机：监测+模式切换+控制+发送 */
void DJmotor_Receive(FDCAN_RxHeaderTypeDef RxHeader, uint8_t *Rx_data); /* 收包解析 */
void DJmotor_PID_Reload(DJMotorPointer motor, uint8_t which, float kp, float ki, float kd); /* 在线调参 */


#endif 