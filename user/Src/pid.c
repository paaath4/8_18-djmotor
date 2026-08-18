#include "pid.h"

// 清空历史
void pid_reset(pidtype *pid)
{
    pid->setval = 0.0f;
    pid->curval = 0.0f;
    pid->outval = 0.0f;

    pid->err = 0.0f;
    pid->err_p = 0.0f;
    pid->err_pp = 0.0f;
    pid->integral = 0.0f;
}

// 初始化
void pid_init(pidtype *pid, float kp, float ki, float kd, pidmode mode)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->mode = mode;
    pid_reset(pid);

    pid->integrallimit = 0.0f; // 先默认不限幅
    pid->outlimit = 0.0f;
}

// 设置输出限幅
void pid_set_outputlimit(pidtype *pid, float out_limit)
{
    pid->outlimit = out_limit;
}

// pid计算
float pid_calculate(pidtype *pid)
{
    pid->err = pid->setval - pid->curval;
    switch (pid->mode)
    {
    case PIDINC: // 增量式
        pid->outval = pid->kp * (pid->err - pid->err_p) +
                      pid->ki * pid->err +
                      pid->kd * (pid->err - 2.0f * pid->err_p + pid->err_pp);
        break;

    case PIDPOS:                   // 位置式
        pid->integral += pid->err; // 积分

        /* 积分限幅 */
        if (pid->integrallimit > 0.0f)
        {
            if (pid->integral > pid->integrallimit)
                pid->integral = pid->integrallimit;
            if (pid->integral < -pid->integrallimit)
                pid->integral = -pid->integrallimit;
        }

        pid->outval = pid->kp * pid->err + pid->ki * pid->integral + pid->kd * (pid->err - pid->err_p);
        break;

    default:
        break;
    }

    pid->err_pp = pid->err_p;
    pid->err_p = pid->err;

    /*输出限幅*/
    if (pid->outlimit > 0.0f)
    {
        if (pid->outval > pid->outlimit)
            pid->outval = pid->outlimit;
        if (pid->outval < -pid->outlimit)
            pid->outval = -pid->outlimit;
    }

    return pid->outval;
}
