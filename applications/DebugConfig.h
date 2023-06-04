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

#ifndef __DEBUGCONFIG_H_
#define __DEBUGCONFIG_H_


/* MSH编码器调试任务 */
#define ENCODERTASKCMD              0

/* MSH电机调试任务 */
#define PWMTASKCMD                  0

/* MSH手柄调试任务 */
#define PS2TASKCMD                  0

/* 电机输出增加电机死区 */
#define MOTORDEADZONE               0

/* 手柄调节参数总开关 */
#define PS2PIDPARAMETER             0

/* 手柄调节直立环PID参数 */
#define VERTICALPIDPARAMETER        1

/* 手柄调节速度环PID参数 */
#define VELOCITYPIDPARAMETER        1

/* 使能转向环 */
#define TURNEN                      1

/* 转向环不根据手柄红绿运行模式判断 */
#define TURNWITHOUTLIGHTMODE        1

/* 使能电机内环 */
#define MOTOREN                     1

/* 打印任务是否采用上位机 */
#define PRINTWITHOUTTOOL            1

/* 打印IMU解算后数据 */
#define PRINTMPUDATA                0

/* 打印卡尔曼滤波后角度数据 */
#define PRINTFILTERANGLE            0

/* 打印编码器速度 */
#define PRINTENCODER                0

/* 打印手柄状态 */
#define PRINTPS2KEY                 0

#endif	/* __DEBUGCONFIG_H_ */
