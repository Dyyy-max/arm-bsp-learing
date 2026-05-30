#ifndef PID_H
#define PID_H

#include <stdint.h>

// 增量式 PID 结构体
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float target;
    float feedback;
    float err;
    float err_last;
    float err_prev;

    float output;
} PID_HandleTypeDef;

// 函数声明
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, float target);
float PID_Calc(PID_HandleTypeDef *pid, float feed_val);

#endif










