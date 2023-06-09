/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-27     Jie       the first version
 */
#ifndef APPLICATIONS_FILTER_KALMAN_H_
#define APPLICATIONS_FILTER_KALMAN_H_

#define Q_angle     0.001       ////角度过程噪声的协方差
#define Q_gyro      0.003       ////角速度过程噪声的协方差
#define R_angle     0.03            ////测量噪声的协方差（即是测量偏差）
#define dt          0.005        ////卡尔曼滤波采样频率
#define C_0         1           ////{1,1,1}; H矩阵的一个观测参数 是常数


/**************卡尔曼运算变量定义**********************
*
***由于卡尔曼为递推运算，结构体需定义为全局变量
***在实际运用中只需定义一个KalmanCountData类型的变量即可
***无需用户定义多个中间变量，简化函数的使用
*/
typedef struct
{
    float   Q_bias;             ////最优估计值的偏差，即估计出来的陀螺仪的漂移量
    float   Angle_err;          ////实测角度与陀螺仪积分角度的差值
    float   PCt_0;
    float   PCt_1;
    float   E;                  ////计算的过程量
    float   K_0;                ////含有卡尔曼增益的另外一个函数，用于计算最优估计值
    float   K_1;                ////含有卡尔曼增益的函数，用于计算最优估计值的偏差
    float   t_0;
    float   t_1;
    float   Pdot[4];            ////Pdot[4] = {0,0,0,0};过程协方差矩阵的微分矩阵
    float   PP[2][2];           //// PP[2][2] = { { 1, 0 },{ 0, 1 } };协方差(covariance)
    float   Angle_Final;        ////后验估计最优角度值（即系统处理最终值）
    float   Gyro_Final;         ////后验估计最优角速度值
}KalmanFilter;


void Kalman_Filter(float Accel, float Gyro, KalmanFilter * Kalman_Struct);
void Kalman_Filter_Init(KalmanFilter * Kalman_Struct);


#endif /* APPLICATIONS_FILTER_KALMAN_H_ */
