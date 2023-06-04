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

/* Drive Function Test */
#if ENCODERTASKCMD
#include <rtthread.h>
#include <rtdevice.h>

#define PULSE_ENCODER_DEV_NAME "pulse3" /* 脉冲编码器名称 */

static int pulse_encoder_sample(int argc, char *argv[])
{
    rt_err_t ret = RT_EOK;
    rt_device_t pulse_encoder_dev = RT_NULL;   /* 脉冲编码器设备句柄 */
    rt_uint32_t index;
    rt_int32_t count;

    /* 查找脉冲编码器设备 */
    pulse_encoder_dev = rt_device_find(PULSE_ENCODER_DEV_NAME);
    if (pulse_encoder_dev == RT_NULL)
    {
        rt_kprintf("pulse encoder sample run failed! can't find %s device!\n", PULSE_ENCODER_DEV_NAME);
        return RT_ERROR;
    }

    /* 以只读方式打开设备 */
    ret = rt_device_open(pulse_encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_DEV_NAME);
        return ret;
    }

    /*for (index = 0; index <= 100; index ++)*/
    for(;;)
    {
        rt_thread_mdelay(50);
        /* 读取脉冲编码器计数值 */
        rt_device_read(pulse_encoder_dev, 0, &count, 1);
        /* 清空脉冲编码器计数值 */
        rt_device_control(pulse_encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
        rt_kprintf("get count %d\n",count);
    }

    rt_device_close(pulse_encoder_dev);
    return ret;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(pulse_encoder_sample, pulse encoder sample);
#endif
