/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-23     Jie       the first version
 */

#ifndef APPLICATIONS_MPU6050_MPU6050_H_
#define APPLICATIONS_MPU6050_MPU6050_H_

#include "Kalman.h"

#define MPU6050_I2C_BUS_NAME        "i2c1"  /* 传感器连接的I2C总线设备名称 */
#define RT_I2C_WR                   0x0000
#define RT_I2C_RD                   (1u << 0)

#define PI 3.14159265

struct rt_my_i2c_msg
{
    rt_uint16_t addr;
    rt_uint16_t flags;
    rt_uint16_t len;
    rt_uint8_t  *buf;
};

struct mpu6050_data{
    short aacx;
    short aacy;
    short aacz;                     /* 加速度传感器原始数据 */

    short gyrox;
    short gyroy;
    short gyroz;                    /* 陀螺仪原始数据 */

    short temper;                   /* 温度 */

    float accl_pitch;               /* 加速度计俯仰角 */
    float accl_yaw;                 /* 加速度计偏航角 */
    float accl_roll;                /* 加速度计翻滚角 */

    float gyro_pitch;               /* 陀螺仪俯仰角 */
    float gyro_yaw;                 /* 陀螺仪偏航角 */
    float gyro_roll;                /* 陀螺仪翻滚角 */
};

struct mpu6050_offset{
    short gyrox;
    short gyroy;
    short gyroz;                    /* 陀螺仪偏移量 */

    short acclx;
    short accly;
    short acclz;                    /* 加速度计偏移量 */
};

struct IMU_OutData
{
    rt_int16_t   Angle_Final;       /* 小车当前角度 */
    rt_int16_t   Gyro_Final;        /* 无效数据 */
    rt_int16_t   gyrox;             /* 小车翻滚角角速度 */
};

void IMU_thread_entry(void);
void IMU_thread_task(void);

extern void Kalman_Filter(float Accel, float Gyro, KalmanFilter * Kalman_Struct);
extern void Kalman_Filter_Init(KalmanFilter * Kalman_Struct);

#endif /* APPLICATIONS_MPU6050_MPU6050_H_ */
