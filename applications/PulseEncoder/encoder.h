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

#ifndef __ENCODER_H_
#define __ENCODER_H_

#define LEFT_TIRE						TIM3
#define LEFT_TIRE_HANDLE		        htim3
#define RIGHT_TIRE	                    TIM4
#define RIGHT_TIRE_HANDLE		        htim4

typedef struct
{
    rt_int16_t leftWheel;           /* 左轮速度 */
    rt_int16_t righWheel;           /* 右轮速度 */
} VehicleSpeed;

void Encoder_thread_entry(void);

#endif	/* __ENCODER_H_ */
