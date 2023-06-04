/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-27     Jie          the first version
 * 2023-05-12     Zeng Jie     add function code and comments
 *
 * 程序清单：这是一个脉冲编码器设备使用例程
 * 例程导出了 pulse_encoder_sample 命令到控制终端
 * 命令调用格式：pulse_encoder_sample
 * 程序功能：每隔 500 ms 读取一次脉冲编码器外设的计数值，然后清空计数值，将读取到的计数值打印出来。
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <encoder.h>
#include <DebugConfig.h>

#define PULSE_ENCODER_DEV_NAME_Lift     "pulse3"        /* 脉冲编码器名称 */
#define PULSE_ENCODER_DEV_NAME_Right    "pulse4"        /* 脉冲编码器名称 */

static rt_device_t lift_pulse_encoder_dev = RT_NULL;    /* 脉冲编码器设备句柄 */
static rt_device_t right_pulse_encoder_dev = RT_NULL;   /* 脉冲编码器设备句柄 */

VehicleSpeed vehicleSpeed = {0};                        /* 车辆速度 */

extern rt_sem_t Speed_DATA_sem;

static int EcoderInit(void);
static void GetEcoderValue(VehicleSpeed *Vehicle);

void Encoder_thread_entry(void)
{
    rt_err_t ret = RT_ERROR;

    ret = EcoderInit();

    while (RT_ERROR == ret);

    for (;;)
    {
        ret = rt_sem_trytake(Speed_DATA_sem);
        if (RT_EOK == ret)
        {
            GetEcoderValue(&vehicleSpeed);

            rt_sem_release(Speed_DATA_sem);
        }
        /* Encoder_Task T: 200Hz */
        rt_thread_mdelay(5);
    }
}

static int EcoderInit(void)
{
    rt_err_t ret = RT_ERROR;

    /*左电机*/
    /* 查找脉冲编码器设备 */
    lift_pulse_encoder_dev = rt_device_find(PULSE_ENCODER_DEV_NAME_Lift);
    if (lift_pulse_encoder_dev == RT_NULL)
    {
        rt_kprintf("pulse encoder sample run failed! can't find %s device!\n", PULSE_ENCODER_DEV_NAME_Lift);
        return RT_ERROR;
    }
    /* 以只读方式打开设备 */
    ret = rt_device_open(lift_pulse_encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_DEV_NAME_Lift);
        return ret;
    }

    /*右电机*/
    right_pulse_encoder_dev = rt_device_find(PULSE_ENCODER_DEV_NAME_Right);
    if (right_pulse_encoder_dev == RT_NULL)
    {
        rt_kprintf("pulse encoder sample run failed! can't find %s device!\n", PULSE_ENCODER_DEV_NAME_Right);
        return RT_ERROR;
    }
    ret = rt_device_open(right_pulse_encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_DEV_NAME_Right);
        return ret;
    }

    return RT_EOK;
}

static void GetEcoderValue(VehicleSpeed *Vehicle)
{
    /*Vehicle ->righWheel    =   -(short)count;*/
    rt_int32_t count;

    /* Get Value */
    /* 读取脉冲编码器计数值 */
    rt_device_read(lift_pulse_encoder_dev, 0, &count, 1);
    Vehicle->leftWheel = (short)count;

    rt_device_read(right_pulse_encoder_dev, 0, &count, 1);
    Vehicle->righWheel = -(short)count;

    /* Reset */
    /* 清空脉冲编码器计数值 */
    rt_device_control(lift_pulse_encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
    rt_device_control(right_pulse_encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
}
