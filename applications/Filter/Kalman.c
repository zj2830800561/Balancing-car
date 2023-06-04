/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-27     Jie       the first version
 */

#include <rtthread.h>
#include <stdlib.h>
#include "Kalman.h"

/**
  ******************************************************************************
  * @file    void Kalman_Filter_Init(KalmanCountData * Kalman_Struct)
  * @author  willieon
  * @version V0.1
  * @date    January-2015
  * @brief   卡尔曼滤波计算中间量初始化
  *
  *
  ******************************************************************************
  * @attention
  *
  *
  *
  *
  ******************************************************************************
  */
void Kalman_Filter_Init(KalmanFilter * Kalman_Struct)
{
        Kalman_Struct -> Angle_err      = 0;        //计算中间值 Angle 观测值-预估值
        Kalman_Struct -> Q_bias             = 0;        //陀螺仪飘移预估值
        Kalman_Struct -> PCt_0              = 0;        //计算中间值
        Kalman_Struct -> PCt_1              = 0;
        Kalman_Struct -> E                      = 0;
        Kalman_Struct -> K_0                    = 0;        //K:卡尔曼增益
        Kalman_Struct -> K_1                    = 0;
        Kalman_Struct -> t_0                    = 0;        //t:计算中间变量
        Kalman_Struct -> t_1                    = 0;
        Kalman_Struct -> Pdot[0]            = 0;        //计算P矩阵的中间矩阵
        Kalman_Struct -> Pdot[1]            = 0;
        Kalman_Struct -> Pdot[2]            = 0;
        Kalman_Struct -> Pdot[3]            = 0;
        Kalman_Struct -> PP[0][0]           = 1;
        Kalman_Struct -> PP[0][1]           = 0;
        Kalman_Struct -> PP[1][0]           = 0;
        Kalman_Struct -> PP[1][1]           = 1;
        Kalman_Struct -> Angle_Final    = 0;        //最优估计的角度   是最终角度结果
        Kalman_Struct -> Gyro_Final     = 0;        //最优估计角速度

}


/**
  ******************************************************************************
  * @file    void Kalman_Filter(float Accel,    float Gyro, KalmanCountData * Kalman_Struct)
  * @author  willieon
  * @version V0.1
  * @date    January-2015
  * @brief   卡尔曼滤波计算
  *
  *
  ******************************************************************************
  * @attention
  *                Accel:加速度计数据处理后进来的角度值
  *                Gyro :陀螺仪数据处理后进来的角速度值
  *                Kalman_Struct:递推运算所需要的中间变量，由用户定义为全局结构体变量
  *                Kalman_Struct -> Angle_Final  为滤波后角度最优值
  *                Kalman_Struct -> Gyro_Final   为后验角度值
输入参数中包含了加速度计和陀螺仪的测量值，但是输出的结果是经过滤波之后的最优角度值
  ******************************************************************************
  */
void Kalman_Filter(float Accel, float Gyro, KalmanFilter * Kalman_Struct)
{
    //陀螺仪积分角度（先验估计）
    Kalman_Struct -> Angle_Final += (Gyro - Kalman_Struct -> Q_bias) * dt;

    //先验估计误差协方差的微分
    Kalman_Struct -> Pdot[0] = Q_angle - Kalman_Struct -> PP[0][1] - Kalman_Struct -> PP[1][0];
    Kalman_Struct -> Pdot[1] = - Kalman_Struct -> PP[1][1];
    Kalman_Struct -> Pdot[2] = - Kalman_Struct -> PP[1][1];
    Kalman_Struct -> Pdot[3] = Q_gyro;

    //先验估计误差协方差的积分
    Kalman_Struct -> PP[0][0] += Kalman_Struct -> Pdot[0] * dt;
    Kalman_Struct -> PP[0][1] += Kalman_Struct -> Pdot[1] * dt;
    Kalman_Struct -> PP[1][0] += Kalman_Struct -> Pdot[2] * dt;
    Kalman_Struct -> PP[1][1] += Kalman_Struct -> Pdot[3] * dt;

    //计算角度偏差
    Kalman_Struct -> Angle_err = Accel - Kalman_Struct -> Angle_Final;

    //卡尔曼增益计算
    Kalman_Struct -> PCt_0 = C_0 * Kalman_Struct -> PP[0][0];
    Kalman_Struct -> PCt_1 = C_0 * Kalman_Struct -> PP[1][0];

    Kalman_Struct -> E = R_angle + C_0 * Kalman_Struct -> PCt_0;

    Kalman_Struct -> K_0 = Kalman_Struct -> PCt_0 / Kalman_Struct -> E;
    Kalman_Struct -> K_1 = Kalman_Struct -> PCt_1 / Kalman_Struct -> E;

    //后验估计误差协方差计算
    Kalman_Struct -> t_0 = Kalman_Struct -> PCt_0;
    Kalman_Struct -> t_1 = C_0 * Kalman_Struct -> PP[0][1];

    Kalman_Struct -> PP[0][0] -= Kalman_Struct -> K_0 * Kalman_Struct -> t_0;
    Kalman_Struct -> PP[0][1] -= Kalman_Struct -> K_0 * Kalman_Struct -> t_1;
    Kalman_Struct -> PP[1][0] -= Kalman_Struct -> K_1 * Kalman_Struct -> t_0;
    Kalman_Struct -> PP[1][1] -= Kalman_Struct -> K_1 * Kalman_Struct -> t_1;

    Kalman_Struct -> Angle_Final  += Kalman_Struct -> K_0 * Kalman_Struct -> Angle_err;         //后验估计最优角度值
    Kalman_Struct -> Q_bias       += Kalman_Struct -> K_1 * Kalman_Struct -> Angle_err;       //更新最优估计值的偏差
    Kalman_Struct -> Gyro_Final   = Gyro - Kalman_Struct -> Q_bias;                                                 //更新最优角速度值
}
