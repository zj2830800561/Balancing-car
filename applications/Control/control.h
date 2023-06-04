/*
 * Copyright (c) 2006-2023, Individual developers
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-27     Zeng Jie     first version
 * 2023-05-12     Zeng Jie     add function code and comments
 */

#ifndef __CONTROL_H_
#define __CONTROL_H_

#include <rtthread.h>
#include "encoder.h"
#include "ps2.h"

/* 定时器周期，单位ns */
#define PERIOD  10000

#define MAX_SUM_ERROR 1000   // 定义积分项的最大值

/* 电机方向引脚 */
#define AIN1_PIN    GET_PIN(B, 12)
#define AIN2_PIN    GET_PIN(B, 13)
#define BIN1_PIN    GET_PIN(B, 15)
#define BIN2_PIN    GET_PIN(B, 14)

/* 电机转动方向 */
#define AIN1(x) rt_pin_write(AIN1_PIN, (x) ? PIN_HIGH : PIN_LOW)
#define AIN2(x) rt_pin_write(AIN2_PIN, (x) ? PIN_HIGH : PIN_LOW)
#define BIN1(x) rt_pin_write(BIN1_PIN, (x) ? PIN_HIGH : PIN_LOW)
#define BIN2(x) rt_pin_write(BIN2_PIN, (x) ? PIN_HIGH : PIN_LOW)

/*电机死区*/
#define LiftDeadZone    1000
#define RightDeadZone   800

/* 最大倾斜角度 */
#define MAXIMUM_TILT_ANGLE      35
/* PS2手柄遥控速度 */
#define VELOCITY_MIN_TARGET 0
#define VELOCITY_MAX_TARGET 80
/* 该参数上限由手柄模拟值决定，手柄单方向最大值为128，设置为130使手柄遥控不会让小车失控 */
#define VELOCITY_SET_TARGET(x) \
    ((x) > 130 ? 0 : ((x) < 0 ? (x) / 130.0 * VELOCITY_MAX_TARGET * 0.8 : (x) / 130.0 * VELOCITY_MAX_TARGET * 0.8))


/* PS2手柄遥控方向 */
#define TURN_MIN_TARGET 0
#define TURN_MAX_TARGET 50
#define TURN_SET_TARGET(x)  if(130 < x)                             \
                                TURN_MAX_TARGET / 130 * 0.8         \
                            else if(0 > x)                          \
                                0                                   \
                            else                                    \
                                x * TURN_MAX_TARGET / 130 * 0.8

#define IF_THE_INTEGRAL_SEPARATION  0   
//#define IF_THE_INTEGRAL_SEPARATION  1   //是否积分分离  0-不分离，1 -分离

typedef struct
{
    double SetPoint;                // 设定目标 Desired Value
    double kp, ki, kd;
    double Proportion;              // 比例常数 Proportional Const
    double Integral;                // 积分常数 Integral Const
    double Derivative;              // 微分常数 Derivative Const
    double LastError;               // Error[-1]
    double PrevError;               // Error[-2]
    double SumError;                // Sums of Errors
} PID;

#if IF_THE_INTEGRAL_SEPARATION            //是否积分分离预编译开始

double PIDCalc(double NextPoint ,double SepLimit, PID *pp);   //带积分分离的PID运算

#else

double PIDCalc( double NextPoint, PID *pp);     //不带积分分离的PID运算

#endif        //是否积分分离预编译结束


void Control_thread_entry(void);

#endif /* __CONTROL_H_ */
