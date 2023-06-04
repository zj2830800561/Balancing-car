/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-27     RT-Thread    first version
 * 2023-05-12     Zeng Jie     add function code and comments
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include <rtdevice.h>
#include "drv_common.h"
#include "mpu6050.h"
#include "message.h"
#include "encoder.h"
#include "control.h"


#define LED0_PIN    GET_PIN(C, 13)


/* MPU6050数据锁 */
rt_sem_t MPU_DATA_sem = RT_NULL;
/* 卡尔曼滤波器数据锁 */
rt_sem_t Filter_DATA_sem = RT_NULL;
/* 编码器数据锁 */
rt_sem_t Speed_DATA_sem = RT_NULL;
/* PS2按键数据锁 */
rt_sem_t Direction_DATA_sem = RT_NULL;

/*Create task handler*/
static rt_thread_t tid_Create = RT_NULL;
/*MPU6050 task handler*/
static rt_thread_t tid_IMU = RT_NULL;
/*Encoder task handler*/
static rt_thread_t tid_Encoder = RT_NULL;
/*Control task handler*/
static rt_thread_t tid_Control = RT_NULL;
/*Remote task handler*/
static rt_thread_t tid_Remote = RT_NULL;
/*Print task handler*/
static rt_thread_t tid_Print = RT_NULL;

static rt_int8_t SemaphoreInit(void);
static void ThreadCreate(void);


int main(void)
{
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    /* Semaphore init function */
    SemaphoreInit();

    /* Create the function used to create the task */
    tid_Create = rt_thread_create("Create_Thread",
                    (void *)ThreadCreate, RT_NULL,
                    512, 5, 5);

    /* Start */
    if (tid_Create != RT_NULL)
        rt_thread_startup(tid_Create);

    /* LED blink 0.5s frequency */
    for (;;)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }

    /* It will not execute here. */
    return RT_EOK;
}

static rt_int8_t SemaphoreInit(void)
{
    /* 创建一个MPU6050测量值信号量，初始值是 1 */
    MPU_DATA_sem = rt_sem_create("MPU_DATA", 1, RT_IPC_FLAG_PRIO);
    if (MPU_DATA_sem == RT_NULL)
    {
        rt_kprintf("create MPU_DATA semaphore failed.\r\n");
        return -1;
    }
    else
    {
        rt_kprintf("create done. MPU_DATA semaphore value = 1.\r\n");
    }

    /* 滤波器信号量 */
    Filter_DATA_sem = rt_sem_create("FILTER_DATA", 1, RT_IPC_FLAG_PRIO);
    if (MPU_DATA_sem == RT_NULL)
    {
        rt_kprintf("create Filter_DATA semaphore failed.\r\n");
        return -1;
    }
    else
    {
        rt_kprintf("create done. Filter_DATA semaphore value = 1.\r\n");
    }

    /* 车速（编码器）信号量 */
    Speed_DATA_sem = rt_sem_create("Speed_DATA", 1, RT_IPC_FLAG_PRIO);
    if (Speed_DATA_sem == RT_NULL)
    {
        rt_kprintf("create Speed_DATA semaphore failed.\r\n");
        return -1;
    }
    else
    {
        rt_kprintf("create done. Speed_DATA semaphore value = 1.\r\n");
    }

    /* PS2信号量 */
    Direction_DATA_sem = rt_sem_create("Direction_DATA", 1, RT_IPC_FLAG_PRIO);
    if (Direction_DATA_sem == RT_NULL)
    {
        rt_kprintf("create Direction_DATA semaphore failed.\r\n");
        return -1;
    }
    else
    {
        rt_kprintf("create done. Direction_DATA semaphore value = 1.\r\n");
    }

    return RT_EOK;
}

static void ThreadCreate(void)
{
    /* USER CODE BEGIN Init */

    LOG_D("Hello RT-Thread!");
    rt_thread_mdelay(500);

    /* USER CODE END Init */

    /*  MPU6050_Task
     * PRIORITY(0 <- 32): 8
     * STACK_SIZE: 512
     * TIMESLICE: 5
    */
    tid_IMU = rt_thread_create("MPU6050_Thread",
                    (void *)IMU_thread_entry, RT_NULL,
                    1024, 8, 5);

    if (tid_IMU != RT_NULL)
        rt_thread_startup(tid_IMU);


    /*  Encoder_Task
     * PRIORITY(0 <- 32): 8
     * STACK_SIZE: 512
     * TIMESLICE: 5
    */
    tid_Encoder = rt_thread_create("Encoder_Thread",
                    (void *)Encoder_thread_entry, RT_NULL,
                    512, 8, 5);

    if (tid_Encoder != RT_NULL)
        rt_thread_startup(tid_Encoder);


    /*  Remote_Task
     * PRIORITY(0 <- 32): 8
     * STACK_SIZE: 512
     * TIMESLICE: 5
    */
    tid_Remote = rt_thread_create("Remote_Thread",
                    (void *)Remote_thread_entry, RT_NULL,
                    512, 8, 5);

    if (tid_Remote != RT_NULL)
        rt_thread_startup(tid_Remote);


    /*  Print_Task
     * PRIORITY(0 <- 32): 5
     * STACK_SIZE: 512
     * TIMESLICE: 5
    */
    tid_Print = rt_thread_create("Print_Thread",
                    (void *)Print_thread_entry, RT_NULL,
                    512, 5, 5);

    if (tid_Print != RT_NULL)
        rt_thread_startup(tid_Print);
        
    /*  Control_Task
     * PRIORITY(0 <- 32): 8
     * STACK_SIZE: 512
     * TIMESLICE: 5
    */
    tid_Control = rt_thread_create("Control_Thread",
                    (void *)Control_thread_entry, RT_NULL,
                    512, 8, 5);

    if (tid_Control != RT_NULL)
        rt_thread_startup(tid_Control);
}
