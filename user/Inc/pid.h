#ifndef _PID_H_
#define _PID_H_

typedef enum
{
    PIDPOS = 0,/*位置式*/
    PIDINC = 1,/*增量式*/
} pidmode ;

typedef struct 
{
    float kp;
    float ki;
    float kd;

    float setval;//设定
    float curval;//当前
    float outval;//输出

    float err;//当前误差
    float err_p;//上次误差
    float err_pp;//上上次

    float integral;//积分累加
    float integrallimit;//积分限幅(0为不限幅)
    float outlimit;//输出限幅（同上）

    pidmode mode;
} pidtype,*pidpointer;

//初始化pid(设定pid模式，清空历史)
void pid_init(pidtype *pid,float kp, float ki, float kd ,pidmode mode);

//pid计算
float pid_calculate(pidtype *pid);

//清空历史
void pid_reset(pidtype *pid);

//设置输出限幅
void pid_set_outputlimit(pidtype *pid, float out_limit);

#endif