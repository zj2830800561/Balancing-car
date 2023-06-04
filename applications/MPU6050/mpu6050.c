/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-23     Jie          the first version
 * 2023-05-12     Zeng Jie     add function code and comments
 */

#include <rtthread.h>

#include <math.h>
#include "drv_soft_i2c.h"
#include "mpu6050.h"

KalmanFilter filter = {0};                          /* 卡尔曼滤波器 */
struct IMU_OutData IMU_Out = {0};                   /* 供外部使用的数据 */

rt_uint8_t IMUFirstRun = RT_EOK;                    /* 首次运行标志 */
static struct mpu6050_data MPU_Data = {0};          /* 传感器数据 */
static struct mpu6050_offset MPU_Offset = {0};      /* 传感器偏移量 */

static struct rt_i2c_bus_device *device;

static rt_uint8_t MPU_Init(void);
#if 0
static rt_uint8_t MPU_Get_Temperature(void);
#endif  /* Not used */
static rt_uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
static rt_uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az);
static void Calc_To_Angle(struct mpu6050_data * MPU_Out, struct mpu6050_offset * MPU_Offset);
static void MPU_Set_Offset(struct mpu6050_offset * Offset, short AcclX, short AcclY, short AcclZ, short GyroX, short GyroY, short GyroZ);

extern rt_sem_t MPU_DATA_sem;
extern rt_sem_t Filter_DATA_sem;

void IMU_thread_entry(void)
{
    rt_err_t ret = RT_ERROR;

    ret = MPU_Init();

    while(RT_ERROR == ret);

    MPU_Set_Offset(&MPU_Offset, 0, 0, 0, 0, 0, 0);

    Kalman_Filter_Init(&filter);

    IMU_thread_task();
}

void IMU_thread_task(void)
{
    rt_err_t ret = RT_ERROR;

    for (;;)
    {
        ret = rt_sem_trytake(MPU_DATA_sem);
        /* 成功获取信号量（数据的修改权） */
        if(RT_EOK == ret)
        {
            /*获取加速度计和陀螺仪寄存器值*/
            MPU_Get_Gyroscope(&MPU_Data.gyrox,&MPU_Data.gyroy,&MPU_Data.gyroz);
            MPU_Get_Accelerometer(&MPU_Data.aacx, &MPU_Data.aacy, &MPU_Data.aacz);

            /*计算角度变化，包含零飘的修正*/
            Calc_To_Angle(&MPU_Data, &MPU_Offset);

            /*将角度变化值进行滤波处理*/
            Kalman_Filter(MPU_Data.accl_pitch, MPU_Data.gyro_pitch, &filter);

            /* 写入IMU传感器数据 */
            ret = rt_sem_trytake(Filter_DATA_sem);
            if(RT_EOK == ret)
            {
                IMU_Out.Angle_Final = filter.Angle_Final;   /* 最后用于使用的角度值 */
                IMU_Out.gyrox = MPU_Data.gyrox;             /* 翻滚角的角速度，用于抑制小车超调，直立环D控制器 */
                rt_sem_release(Filter_DATA_sem);
            }

            /* 释放信号量 */
            rt_sem_release(MPU_DATA_sem);
        }
        /* IMU_Task T：200Hz */
        rt_thread_mdelay(5);
    }
}


static rt_err_t mpu6050_write_reg(struct rt_i2c_bus_device *dev, rt_uint8_t reg, rt_uint8_t data)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[2] = {reg, data};
    msgs.addr  = 0x68;
    msgs.flags = RT_I2C_WR;
    msgs.buf   = buf;
    msgs.len   = 2;

    if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev, &msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

static rt_err_t mpu6050_read_regs(struct rt_i2c_bus_device *dev, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs[2];
    msgs[0].addr  = 0x68;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    msgs[1].addr  = 0x68;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = buf;
    msgs[1].len   = len;

    if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev, msgs, 2) == 2)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

static rt_uint8_t MPU_Init(void)
{
    rt_uint8_t deviceADDR = 0;
    /*rt_err_t result = RT_NULL;*/

     /*查找I2C总线设备，获取I2C总线设备句柄*/
    device = (struct rt_i2c_bus_device *)rt_i2c_bus_device_find(MPU6050_I2C_BUS_NAME);
    if (device == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", MPU6050_I2C_BUS_NAME);
        return RT_ERROR;
    }
    else
    {
        rt_kprintf("i2c OK!\r\n\r\n");
    }

    /*MPU_Init();*/
    mpu6050_write_reg(device, 0x6B, 0x80);    //复位MPU6050
    rt_thread_mdelay(100);
    mpu6050_write_reg(device, 0x6B, 0x00);    //唤醒MPU6050
    rt_thread_mdelay(100);

    mpu6050_write_reg(device, 0x1B, 3<<3);    //陀螺仪传感器,±2000dps
    mpu6050_write_reg(device, 0x1C, 0<<3);    //加速度传感器,±2g
    mpu6050_write_reg(device, 0x19, 1000/49);    //设置数字低通滤波器
    mpu6050_write_reg(device, 0x1A, 4);    //设置LPF为采样率的一半
    mpu6050_write_reg(device, 0x38, 0);    //关闭所有中断
    mpu6050_write_reg(device, 0x6A, 0);    //I2C主模式关闭
    mpu6050_write_reg(device, 0x23, 0);    //关闭FIFO
    mpu6050_write_reg(device, 0x37, 0x80);    //INT引脚低电平有效

    mpu6050_read_regs(device, 0x75, 1, &deviceADDR);
    rt_kprintf("\r\nMPU6050 ID:0x%2x\r\n", deviceADDR);
    if(0x68 == deviceADDR)
    {
        mpu6050_write_reg(device, 0x68, 0x01);  //设置CLKSEL,PLL X轴为参考
        mpu6050_write_reg(device, 0x6C, 0x00);  //加速度与陀螺仪都工作
        mpu6050_write_reg(device, 0x19, 1000/49);    //设置数字低通滤波器
        mpu6050_write_reg(device, 0x1A, 4);    //设置LPF为采样率的一半
    }
    return RT_EOK;
}

/**
 * This function gets mpu6050 temperature. Not used.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
#if 0
static rt_uint8_t MPU_Get_Temperature(void)
{
    unsigned char  buf[2];
    short raw;
    float temp;
    rt_uint8_t ret = 0;

    mpu6050_read_regs(device, 0x41, 2, buf);

    raw=(buf[0]<<8)| buf[1];
    temp=(36.53+((double)raw)/340)*100;
    /*temp = (long)((35 + (raw / 340)) * 65536L);*/
    /*temp /= 100.0f;*/
    ret = (rt_uint8_t)(temp / 100);

    return ret;
}
#endif  /* Not used */

static rt_uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    rt_uint8_t buf[6],res;

    res=mpu6050_read_regs(device, 0x43, 6, buf);
    if(RT_EOK == res)
    {
        *gx=((rt_uint16_t)buf[0]<<8)|buf[1];
        *gy=((rt_uint16_t)buf[2]<<8)|buf[3];
        *gz=((rt_uint16_t)buf[4]<<8)|buf[5];

        /* printf register raw data */
        /*rt_kprintf("reg data:\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
                buf[0], buf[1],
                buf[2], buf[3],
                buf[4], buf[5]);
        */
    }
    else {
        rt_kprintf("Read Gyro Data Error!\r\n");
        return RT_ERROR;
    }

    /* printf gyro raw data */
    /*
    rt_kprintf("MPU6050 rawX:%d\t\trawY:%d\t\trawZ:%d\r\n",
            (int16_t)(((uint16_t)buf[0]<<8)|buf[1]),
            (int16_t)(((uint16_t)buf[2]<<8)|buf[3]),
            (int16_t)(((uint16_t)buf[4]<<8)|buf[5]));
    */
    return res;
}

static rt_uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    rt_uint8_t buf[6],res;

    /* 0x3B:MPU_ACCEL_XOUTH_REG */
    res = mpu6050_read_regs(device,0x3B,6,buf);

    if(RT_EOK == res)
    {
        *ax=((rt_uint16_t)buf[0]<<8)|buf[1];
        *ay=((rt_uint16_t)buf[2]<<8)|buf[3];
        *az=((rt_uint16_t)buf[4]<<8)|buf[5];
    }
    return res;
}

/* 用数据计算角度的估价值 */
static void Calc_To_Angle(struct mpu6050_data * MPU_Out, struct mpu6050_offset * MPU_Offset)
{
    MPU_Out->accl_pitch = atan2(MPU_Out->aacz,  MPU_Out->aacy) * 180 / PI;                      /* 俯仰角 */
    MPU_Out->accl_yaw   = atan2(MPU_Out->aacy,  MPU_Out->aacx) * 180 / PI;                      /* 偏航角 */
    MPU_Out->accl_roll  = atan2(MPU_Out->aacx,  MPU_Out->aacz) * 180 / PI;                      /* 翻滚角 */

    if(RT_ERROR != IMUFirstRun)
    {
        /* 首次运行假设为完全理想直立状态 */
        MPU_Out->gyro_pitch =   90;
        MPU_Out->gyro_yaw   =   90;
        MPU_Out->gyro_roll  =   90;

        IMUFirstRun = RT_ERROR;
    }
    else
    {
        MPU_Out->gyro_pitch -= (float)(((MPU_Out->gyrox + MPU_Offset->gyrox) / 16.384) * 0.02);                 /* 俯仰角 */
        MPU_Out->gyro_yaw   += (float)(((MPU_Out->gyroy + MPU_Offset->gyroy) / 16.384) * 0.02);                 /* 偏航角 */
        MPU_Out->gyro_roll  += (float)(((MPU_Out->gyroz + MPU_Offset->gyroz) / 16.384) * 0.02);                 /* 翻滚角 */
    }

}

/* 设置MPU6050偏移量 */
static void MPU_Set_Offset(struct mpu6050_offset * Offset,
                    short AcclX, short AcclY, short AcclZ,
                    short GyroX, short GyroY, short GyroZ)
{
    Offset->acclx   =   AcclX;
    Offset->accly   =   AcclY;
    Offset->acclz   =   AcclZ;

    Offset->gyrox   =   GyroX;
    Offset->gyroy   =   GyroY;
    Offset->gyroz   =   GyroZ;
}
MSH_CMD_EXPORT(IMU_thread_entry, pwm device sample);
