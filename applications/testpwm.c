/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-13     desktop      the first version
 * 2023-05-12     Zeng Jie     add function code and comments
 *
 * 程序清单：这是一个PWM设备使用例程
 * 例程导出了 pwm_test 命令到控制终端
 * 命令调用格式：pwm_test
 * 程序功能：定时器2通道3对应的电机将会以50%的占空比进行输出，可用示波器或观察轮子转动情况。
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <DebugConfig.h>

/* Drive Function Test */
#if PWMTASKCMD

/* 设置log文件 */
#define DBG_TAG "pwm_test.c"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define PWM_DEV_NAME "pwm2" /* PWM 设 备 名 称 */
#define PWM_DEV_CHANNEL3 3
#define PWM_DEV_CHANNEL4 4

struct rt_device_pwm *pwm_dev; /* PWM 设 备 句 柄 */

static int pwm_test(void)
{
    rt_uint32_t period, pulse;
    /* 设置周期及脉冲长度 */
    period = 200000;
    pulse = 100000;
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME); // 查找PWM设备
    /* 配置PWM */
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL3, period, pulse); // 占空比应该是50%
    if (rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL3) == RT_EOK)
    {
        LOG_D("PWM Init is ok \t\n");
        return RT_EOK;
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(pwm_test, pwm device sample);
#endif
