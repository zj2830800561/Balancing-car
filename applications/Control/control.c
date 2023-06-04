/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     desktop      the first version
 * 2023-05-12     Zeng Jie     add function code and comments
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "DebugConfig.h"
#include "drv_common.h"
#include "control.h"
#include "mpu6050.h"
#include "encoder.h"
#include "ps2.h"

/* 设置log文件 */
#define DBG_TAG "pwm_test.c"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define PWM_DEV_NAME                "pwm2"      /* PWM 设备名称 */
#define PWM_DEV_CHANNEL_LEFT        3
#define PWM_DEV_CHANNEL_RIGHT       4

struct rt_device_pwm *pwm_dev;                  /* PWM 设备句柄 */
static PID MotorHead[2] = {0};                  /* 电机控制块    */
static PID VerticalHead = {0};                  /* 直立环控制块 */
static PID VelocityHead = {0};                  /* 速度环控制块 */
static PID TurnHead = {0};                      /* 转向环控制块 */
static rt_uint16_t SpeedControlCount = 0;       /* 转向环计时器 */

static rt_int16_t PIN_Init(void);
static rt_int16_t PWM_Init(void);
static rt_int16_t PID_Init(void);
static rt_int16_t HeadInit(double SetPoint, double kp, double ki, double kd, PID *pp);
static rt_int16_t PS2KeyExplanationToVelocity(PS2_Light_Mode_t PS2LightModeValue, PS2_keys_data_t PS2KeysValue, PID *pp);
static rt_int16_t VelocityTargetSet(double Target, PID *pp);
static rt_int16_t Vertical(PID *pp, rt_int16_t Angle, rt_int16_t GyroValueX);
static rt_int16_t Velocity(PID *pp, VehicleSpeed *vehicleSpeed);
#if TURNEN
#if TURNWITHOUTLIGHTMODE
static rt_int8_t Turn(PID *pp, PS2_keys_data_t PS2KeysValue, rt_int16_t PID_Value, VehicleSpeed *TargetValue);
#else
static rt_int8_t Turn(PID *pp, PS2_Light_Mode_t PS2LightModeValue, PS2_keys_data_t PS2KeysValue, rt_int16_t PID_Value, VehicleSpeed *OutputValue);
static rt_int16_t PS2KeyExplanationToTurn(PID *pp, PS2_Light_Mode_t PS2LightModeValue, PS2_keys_data_t PS2KeysValue, rt_int16_t PID_Value, VehicleSpeed *OutputValue);
#endif  /* TURNWITHOUTLIGHTMODE */
#endif  /* TURNEN */
#if MOTOREN
static rt_int16_t Motor(PID *Lpp, PID *Rpp, VehicleSpeed *vehicleSpeed, VehicleSpeed *targetSpeed, VehicleSpeed *OutputValue);
#endif  /* MOTOREN */
static rt_int16_t setOutput(VehicleSpeed *OutputValue);
static rt_int16_t RunningDetecte(float FinalAngle, VehicleSpeed *OutputValue);
#if PS2PIDPARAMETER
static void PS2KeyExplanationToPID(PS2_Light_Mode_t PS2LightModeValue, PS2_keys_data_t PS2KeysValue, PID *Verticalpp, PID *Velocitypp);
#endif  /* PS2PIDPARAMETER */

extern rt_sem_t Filter_DATA_sem;
extern KalmanFilter filter;
extern struct IMU_OutData IMU_Out;
extern rt_sem_t Speed_DATA_sem;
extern VehicleSpeed vehicleSpeed;
extern rt_sem_t Direction_DATA_sem;
extern rt_err_t PS2ConnectedFlag;
extern PS2_Light_Mode_t PS2WorkMode;
extern PS2_keys_data_t PS2_Keys;
extern rt_uint8_t lx_rocker_offset;
extern rt_uint8_t ly_rocker_offset;
extern rt_uint8_t rx_rocker_offset;
extern rt_uint8_t ry_rocker_offset;

void Control_thread_entry(void)
{
    rt_err_t ret = RT_ERROR;
    rt_int16_t FinalAngle = 0;                      /* data get from MPU6050_Task IMU */
    short gyrox = 0;                                /* data get from MPU6050_Task IMU */
    VehicleSpeed VehicleSpeedValue = {0};           /* data get from Encoder_Task VehicleSpeed*/
    VehicleSpeed TargetSpeedValue = {0};            /* Velocity inner loop calculation variables */
    VehicleSpeed OutputValue = {0};                 /* Output Value */
    PS2_Light_Mode_t PS2LightModeValue = 0;         /* data get from Remote_Task PS2LightMode */
    PS2_keys_data_t PS2KeysValue = {0};             /* data get from Remote_Task PS2Keys */
    rt_uint16_t PID_Value = 20, VerticalRet = 0, VelocityRet = 0;

    while (RT_ERROR == PIN_Init());
    while (RT_ERROR == PWM_Init());
    while (RT_ERROR == PID_Init());

    for (;;)
    {
        /* 1. get final angle from MPU6050_Task */
        ret = rt_sem_trytake(Filter_DATA_sem);
        if (RT_EOK == ret)
        {
            FinalAngle = IMU_Out.Angle_Final;
            gyrox = IMU_Out.gyrox;
            rt_sem_release(Filter_DATA_sem);
        }


        /* 2. get vehicle speed from Encoder_Task */
        ret = rt_sem_trytake(Speed_DATA_sem);
        if (RT_EOK == ret)
        {
            rt_memcpy(&VehicleSpeedValue, &vehicleSpeed, sizeof(vehicleSpeed));
            rt_sem_release(Speed_DATA_sem);
        }


        /* 3. get direction from Remote_Task */
        ret = rt_sem_trytake(Direction_DATA_sem);
        if (RT_EOK == ret)
        {
            /* PS2 connecteded */
            if (RT_EOK == PS2ConnectedFlag)
            {
                /* running mode */
                PS2LightModeValue = PS2WorkMode;

                /* key value */
                rt_memcpy(&PS2KeysValue, &PS2_Keys, sizeof(PS2_Keys));
            }
            rt_sem_release(Direction_DATA_sem);
        }
        /* Parsing keys
         * The direction of the steering ring and the target speed of the speed ring are involved.
         * Only the speed loop is parsed here, the direction is parsed by the steering loop itself
        */
        PS2KeyExplanationToVelocity(PS2LightModeValue, PS2KeysValue, &VelocityHead);

        /* **********************************************************
         * Debug Process            
         * Use to PS2 control PID parameter
           **********************************************************
        */
#if PS2PIDPARAMETER
        PS2KeyExplanationToPID(PS2LightModeValue, PS2KeysValue, &VerticalHead, &VelocityHead);
#endif  /* PS2PIDPARAMETER */
        /* ******************************************************** */


        /* 4. vertical */
        VerticalRet   =   Vertical(&VerticalHead, FinalAngle, gyrox);


        /* 5. velocity */
        SpeedControlCount += 1;         /* count plus one */

        /* Adjusting the speed loop every 40ms minimizes the impact on the upright loop */
        if(4 == SpeedControlCount)
        {
            /* reset counter */
            SpeedControlCount = 0;
            /* velocity calculate */
            VelocityRet   =  Velocity(&VelocityHead, &VehicleSpeedValue);
            /* Add velocity control effect */
            PID_Value = VerticalRet + VelocityRet;
        }
        else
        {
            /* Removal vertical interface */
            PID_Value = VerticalRet;
        }


        /* 6. turn */
#if TURNEN
#if TURNWITHOUTLIGHTMODE
        Turn(&TurnHead, PS2KeysValue, PID_Value, &TargetSpeedValue);
#else
        Turn(&TurnHead, PS2LightModeValue, PS2KeysValue, PID_Value, &TargetSpeedValue);
#endif  /* TURNWITHOUTLIGHTMODE */
#else
        TargetSpeedValue.leftWheel = PID_Value;
        TargetSpeedValue.righWheel = PID_Value;
#endif  /* TURNEN */


        /* 7. motor */
#if MOTOREN
        Motor(&MotorHead[0], &MotorHead[1], &VehicleSpeedValue, &TargetSpeedValue, &OutputValue);
#else
        OutputValue.leftWheel = TargetSpeedValue.leftWheel;
        OutputValue.righWheel = TargetSpeedValue.righWheel;
#endif  /* MOTOREN */


        /* 8. detecte running status */
        RunningDetecte(FinalAngle, &OutputValue);


        /* 9. output PWM */
        setOutput(&OutputValue);


        /* Control_Task T: 100Hz */
        rt_thread_mdelay(10);
    }
}

/* 电机方向引脚，PIN设备初始化 */
static rt_int16_t PIN_Init(void)
{
    rt_pin_mode(AIN1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(AIN2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BIN1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BIN2_PIN, PIN_MODE_OUTPUT);

    return RT_EOK;
}

/* 电机设备初始化 */
static rt_int16_t PWM_Init(void)
{
    rt_err_t ret = RT_ERROR;
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME); // 查找PWM设备
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", PWM_DEV_NAME);
        return RT_ERROR;
    }
    else
    {
        rt_kprintf("PWM OK!\r\n\r\n");
    }

    // 配置PWM
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_LEFT, PERIOD, 0); /* 初始化占空比应该是0% */
    if (rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL_LEFT) == RT_EOK)
    {
        LOG_D("Left Wheel Init is ok \t\n");
        rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_RIGHT, PERIOD, 0); /* 初始化占空比应该是0% */
        if (rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL_RIGHT) == RT_EOK)
        {
            LOG_D("Right Wheel Init is ok \t\n");
            ret = RT_EOK;
        }
    }

    return ret;
}

/* PID控制器参数初始化 */
static rt_int16_t PID_Init(void)
{
    /* 直立环控制块 */
    HeadInit(90, 1350, 0, 3.4, &VerticalHead);

    /* 速度外环控制块 */
    HeadInit(0, 45, 0.225, 0, &VelocityHead);

    /* 转向环控制块 */
    HeadInit(0, 800, 0, 0, &TurnHead);

    /* 电机内环控制块 */
    HeadInit(0, 80, 0.18, 0, &MotorHead[0]);
    HeadInit(0, 70, 0.18, 0, &MotorHead[1]);

    /*return ret;*/
    return RT_EOK;
}

/* 解析按键，设置PID参数值 */
#if PS2PIDPARAMETER
static void PS2KeyExplanationToPID(PS2_Light_Mode_t PS2LightModeValue, PS2_keys_data_t PS2KeysValue, PID *Verticalpp, PID *Velocitypp)
{
    rt_int16_t tempValue = 0;
    /* 未连接状态 */
    if (PS2_LIGHT_MODE_NO_CONNECT == PS2LightModeValue)
        return RT_ERROR;
#if VERTICALPIDPARAMETER
    /* 直立环控制方式：
        Kp+     Kp-     Kd+     Kd-
        L1      L2      R1      R2
     */
    if (0 == PS2KeysValue.key_value[PS2_KEY_L1])
    {
        /* Kp+ */
        Verticalpp->kp += 1;
    }
    else if (0 == PS2KeysValue.key_value[PS2_KEY_L2])
    {
        /* Kp- */
        Verticalpp->kp -= 1;
    }
    rt_kprintf("Verticalpp.Kp = %d\r\n", (uint16_t)Verticalpp->kp);

    if (0 == PS2KeysValue.key_value[PS2_KEY_R1])
    {
        /* Kd+ */
        Verticalpp->kd += 0.01;
    }
    else if (0 == PS2KeysValue.key_value[PS2_KEY_R2])
    {
        /* Kd- */
        Verticalpp->kd -= 0.01;
    }
    rt_kprintf("Verticalpp.Kd = %d\r\n\r\n", (uint16_t)Verticalpp->kd);

    /* **********************************************************
                    Print the PS2 pressed button
       **********************************************************
    */
//    if(PS2KeysValue.key_value[PS2_KEY_L1] == 0){
//        rt_kprintf("L1 is done\r\n\r\n");
//    }
//    if(PS2KeysValue.key_value[PS2_KEY_L2] == 0){
//        rt_kprintf("L2 is done\r\n\r\n");
//    }
//    if(PS2KeysValue.key_value[PS2_KEY_R1] == 0){
//        rt_kprintf("R1 is done\r\n\r\n");
//    }
//    if(PS2KeysValue.key_value[PS2_KEY_R2] == 0){
//        rt_kprintf("R2 is done\r\n\r\n");
//    }
#endif  /* VERTICALPIDPARAMETER */
#if VELOCITYPIDPARAMETER
    /* 速度环控制方式：
        Kp+     Kp-     Ki+     Ki-
        △       X      口      〇
     */
    // if (0 == PS2KeysValue.key_value[PS2_KEY_L1])
    // {
    //     /* Kp+ */
    //     Velocitypp->kp += 0.01;
    //     Velocitypp->ki += 0.01 / 200;
    // }
    // else if (0 == PS2KeysValue.key_value[PS2_KEY_L2])
    // {
    //     /* Kp- */
    //     Velocitypp->kp -= 0.01;
    //     Velocitypp->ki -= 0.01 / 200;
    // }

    if (0 == PS2KeysValue.key_value[PS2_KEY_GREEN])
    {
        /* Kp+ */
        Velocitypp->kp += 1;
    }
    else if (0 == PS2KeysValue.key_value[PS2_KEY_BLUE])
    {
        /* Kp- */
        Velocitypp->kp -= 1;
    }
    rt_kprintf("Velocitypp.Kp = %d\r\n", (uint16_t)Velocitypp->kp);

    if (0 == PS2KeysValue.key_value[PS2_KEY_PINK])
    {
        /* Ki+ */
        Velocitypp->ki += 1 / 200;
    }
    else if (0 == PS2KeysValue.key_value[PS2_KEY_RED])
    {
        /* Ki- */
        Velocitypp->ki -= 1 / 200;
    }
    rt_kprintf("Velocitypp.Ki = %d\r\n\r\n", (uint16_t)Velocitypp->ki);
#endif  /* VELOCITYPIDPARAMETER */
}
#endif  /* PS2PIDPARAMETER */

/* PID控制器参数设置 */
static rt_int16_t HeadInit(double SetPoint, double kp, double ki, double kd, PID *pp)
{
    pp->SetPoint = SetPoint; // 设定目标 Desired Value
    pp->kp = kp;             // 比例常数 Proportional Const
    pp->ki = ki;             // 积分常数 Integral Const
    pp->kd = kd;             // 微分常数 Derivative Const
    pp->Proportion = 0;
    pp->Integral = 0;
    pp->Derivative = 0;
    pp->LastError = 0; // Error[-1]
    pp->PrevError = 0; // Error[-2]
    pp->SumError = 0;  // Sums of Errors

    return RT_EOK;
}

/* 解析按键，设置速度环的目标速度 */
static rt_int16_t PS2KeyExplanationToVelocity(PS2_Light_Mode_t PS2LightModeValue, PS2_keys_data_t PS2KeysValue, PID *pp)
{
    /* 未连接状态 */
    if (PS2_LIGHT_MODE_NO_CONNECT == PS2LightModeValue)
    {
        VelocityTargetSet(VELOCITY_MIN_TARGET, pp);
        return RT_ERROR;
    }

    /* 控制方式：
        前进    后退    左转    右转
        L1      R1      L2      R2
        UP      DOWN    口      〇
            Ly              Rx      （摇杆模拟量，仅支持红灯模式）
     */
    /* 按键按下为0 */
    if (((0 == PS2KeysValue.key_value[PS2_KEY_L1])      &&
         (0 == PS2KeysValue.key_value[PS2_KEY_R1]))         ||
        ((0 == PS2KeysValue.key_value[PS2_KEY_PAD_UP])  &&
         (0 == PS2KeysValue.key_value[PS2_KEY_PAD_DOWN])))
    {
        /* 同时按下无效 */
        VelocityTargetSet(VELOCITY_MIN_TARGET, pp);
    }
    else if ((0 == PS2KeysValue.key_value[PS2_KEY_L1])  || 
             (0 == PS2KeysValue.key_value[PS2_KEY_PAD_UP]))
    {
        /* 前进 */
        VelocityTargetSet(VELOCITY_MAX_TARGET, pp);
    }
    else if ((0 == PS2KeysValue.key_value[PS2_KEY_R1]) || 
             (0 == PS2KeysValue.key_value[PS2_KEY_PAD_DOWN]))
    {
        /* 后退 */
        VelocityTargetSet(-VELOCITY_MAX_TARGET, pp);
    }
    else
    {
        /* 没有按键按下 */
        VelocityTargetSet(VELOCITY_MIN_TARGET, pp);
    }

    /* 使用摇杆模拟量来设置期望速度 */
    if (PS2_LIGHT_MODE_RED == PS2LightModeValue)
    {
        VelocityTargetSet(
                (VELOCITY_SET_TARGET(PS2KeysValue.key_value[PS2_KEY_PAD_UP] - ly_rocker_offset)) * (VELOCITY_MAX_TARGET / 128),
                pp);
    }

    return RT_EOK;
}

/* 设置目标速度 */
static rt_int16_t VelocityTargetSet(double Target, PID *pp)
{
    pp->SetPoint = Target;

    return RT_EOK;
}

/*****************
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：  pp:PID结构体，
        GyroValueX:X轴角速度值
出口：  直立环输出
******************/
static rt_int16_t Vertical(PID *pp, rt_int16_t Angle, rt_int16_t GyroValueX)
{
    rt_int16_t ret;

    /* 计算误差 */
    pp->LastError = pp->SetPoint - Angle;

    /* 计算微分项 */
    pp->Derivative = GyroValueX;

    ret = pp->kp * pp->LastError + pp->kd * pp->Derivative;

    return ret;
}

/*****************
速度环PI控制器：Kp*Ek+Ki*Ek_S(Ek_S：偏差的积分)

入口：  pp:PID结构体，
        vehicleSpeed:小车当前运行速度
出口：  速度环输出
******************/
static rt_int16_t Velocity(PID *pp, VehicleSpeed *vehicleSpeed)
{
    /* Kp = 200 * Ki */
    rt_int16_t ret = 0;

    /* 计算误差 */
    pp->LastError = (vehicleSpeed->leftWheel + vehicleSpeed->righWheel) - pp->SetPoint;

    /* 防止速度突变 */
    pp->LastError = (pp->LastError + 2 * pp->PrevError) / 3;

    /* 状态保存 */
    pp->PrevError = pp->LastError;

    /* 计算积分项 */
    pp->SumError += pp->LastError;

    /* 积分限幅 */
    pp->SumError = (pp->SumError > 10000) ? 10000 : ((pp->SumError < -10000) ? -10000 : pp->SumError);

    ret = (pp->kp * pp->LastError + pp->ki * pp->SumError);

    return ret;
}

/*****************
转向环：

入口：  pp:PID结构体，
        PS2LightModeValue:手柄连接状态，
        PS2KeysValue：手柄按键状态，
        PID_Value：直立环和速度环的输出量之和，
        TargetValue：给电机内环的目标速度输出
出口：  运行状态是否异常
******************/
#if TURNEN
#if TURNWITHOUTLIGHTMODE
/* PS2LightModeValue is unstable */
static rt_int8_t Turn(PID *pp, PS2_keys_data_t PS2KeysValue, rt_int16_t PID_Value, VehicleSpeed *TargetValue)
{
    rt_int16_t turnRange = 0;   /* 转向幅度 */

    /* 控制方式：
        左转    右转    左旋    右旋
        L2      R2      △      ×
        口      〇
     */
    if((0 == PS2KeysValue.key_value[PS2_KEY_L2] && 
        0 == PS2KeysValue.key_value[PS2_KEY_R2])   || 
       (0 == PS2KeysValue.key_value[PS2_KEY_RED] && 
        0 == PS2KeysValue.key_value[PS2_KEY_PINK]) ||
       (0 == PS2KeysValue.key_value[PS2_KEY_GREEN] && 
        0 == PS2KeysValue.key_value[PS2_KEY_BLUE]))
    {
        /* 同时按下无效 */
        turnRange = 0;
    }
    else if(0 == PS2KeysValue.key_value[PS2_KEY_L2] || 
            0 == PS2KeysValue.key_value[PS2_KEY_PINK])
    {
        /* 负数为左转，左转为左旋的四分之一 */
        turnRange = -(pp->kp / 4);
    }
    else if(0 == PS2KeysValue.key_value[PS2_KEY_R2] || 
            0 == PS2KeysValue.key_value[PS2_KEY_RED])
    {
        /* 正数为右转 */
        turnRange = pp->kp / 4;
    }
    else if(0 == PS2KeysValue.key_value[PS2_KEY_GREEN])
    {
        /* 左旋 */
        turnRange = -(pp->kp);
    }
    else if(0 == PS2KeysValue.key_value[PS2_KEY_BLUE])
    {
        /* 右旋 */
        turnRange = pp->kp;
    }
    else
    {
        turnRange = 0;
    }

    TargetValue->leftWheel = PID_Value - turnRange;
    TargetValue->righWheel = PID_Value + turnRange;

    return RT_EOK;
}
#else
static rt_int8_t Turn(PID *pp, PS2_Light_Mode_t PS2LightModeValue, PS2_keys_data_t PS2KeysValue, rt_int16_t PID_Value, VehicleSpeed *OutputValue)
{
    /* 手柄未连接 */
    if (PS2_LIGHT_MODE_NO_CONNECT == PS2LightModeValue)
    {
        OutputValue->leftWheel = PID_Value;
        OutputValue->righWheel = PID_Value;
    }
    else
    {
        /* 如果要转向，直接将targetSpeedValue ± kp */
        /* Hysteresis */
        PS2KeyExplanationToTurn(pp, PS2LightModeValue, PS2KeysValue, PID_Value, OutputValue);
    }
    
    return RT_EOK;
}

/* 解析按键，设置转向环的转动程度 */
static rt_int16_t PS2KeyExplanationToTurn(PID *pp, PS2_Light_Mode_t PS2LightModeValue, PS2_keys_data_t PS2KeysValue, rt_int16_t PID_Value, VehicleSpeed *OutputValue)
{
    float Hysteresis = 0;

    /* 按键按下为0 */
    if (((0 == PS2KeysValue.key_value[PS2_KEY_L2])  && 
        (0 == PS2KeysValue.key_value[PS2_KEY_R2]))      || 
        ((0 == PS2KeysValue.key_value[PS2_KEY_GREEN]) &&
        (0 == PS2KeysValue.key_value[PS2_KEY_BLUE])))
    {
        /* 同时按下无效 */
        OutputValue->leftWheel = PID_Value;
        OutputValue->righWheel = PID_Value;
    }
    else if ((0 == PS2KeysValue.key_value[PS2_KEY_L2])  || 
             (0 == PS2KeysValue.key_value[PS2_KEY_GREEN]))
    {
        /* 左转 */
        OutputValue->leftWheel = PID_Value - pp->kp;
        OutputValue->righWheel = PID_Value + pp->kp;
    }
    else if ((0 == PS2KeysValue.key_value[PS2_KEY_R2])  ||
             (0 == PS2KeysValue.key_value[PS2_KEY_BLUE]))
    {
        /* 右转 */
        OutputValue->leftWheel = PID_Value + pp->kp;
        OutputValue->righWheel = PID_Value - pp->kp;
    }
    else
    {
        /* 没有按键按下 */
        OutputValue->leftWheel = PID_Value;
        OutputValue->righWheel = PID_Value;
    }

    /* 使用摇杆模拟量来设置期望速度 */
    if (PS2_LIGHT_MODE_RED == PS2LightModeValue)
    {
        /* 摇杆模拟量 */
        Hysteresis = PS2KeysValue.key_value[PS2_KEY_ROCKER_RX] - 128;

        if (0 > Hysteresis)
        {
            /* 左转 */
            OutputValue->leftWheel =PID_Value - (abs(Hysteresis) / 130 * pp->kp);
            OutputValue->righWheel =PID_Value + (1 + (abs(Hysteresis) / 130 * pp->kp));
        }
        else
        {
            /* 右转 */
            OutputValue->leftWheel = PID_Value +  (1 + (abs(Hysteresis) / 130 * pp->kp));
            OutputValue->righWheel = PID_Value -  (abs(Hysteresis) / 130 * pp->kp);
        }
    }

    return RT_EOK;
}
#endif  /* TURNWITHOUTLIGHTMODE */
#endif  /* TURNEN */

/*****************
电机速度环PI控制器：Kp*Ek+Ki*Ek_I

入口：  *pp: PID控制结构体(L=left;R=right),
        targetSpeed: 电机期望速度(0-10000),
        OutputValue: 电机输出量
出口：  运行状态是否异常
******************/
#if MOTOREN
static rt_int16_t Motor(PID *Lpp, PID *Rpp, VehicleSpeed *vehicleSpeed, VehicleSpeed *targetSpeed, VehicleSpeed *OutputValue)
{
    /* Vehicle operating speed */
    rt_int16_t leftTemp = 0, rightTemp = 0;

    /* 左轮速度 */
    Lpp->LastError = targetSpeed->leftWheel - vehicleSpeed->leftWheel; // 偏差

    /* 防止速度突变 */
    Lpp->LastError = (Lpp->LastError + 2 * Lpp->PrevError) / 3;

    /* 状态保存 */
    Lpp->PrevError = Lpp->LastError;

    /* 计算积分项 */
    Lpp->SumError += Lpp->LastError;

    /* 积分限幅 */
    Lpp->SumError = (Lpp->SumError > 10000) ? 10000 : ((Lpp->SumError < -10000) ? -10000 : Lpp->SumError);

    leftTemp = Lpp->kp * Lpp->LastError + Lpp->ki * Lpp->SumError; // 比例项 + 积分项

    OutputValue->leftWheel = vehicleSpeed->leftWheel + leftTemp;

    /* 右轮速度 */
    Rpp->LastError = targetSpeed->righWheel - vehicleSpeed->righWheel; // 偏差

    /* 防止速度突变 */
    Rpp->LastError = (Rpp->LastError + 2 * Rpp->PrevError) / 3;

    /* 状态保存 */
    Rpp->PrevError = Rpp->LastError;

    /* 计算积分项 */
    Rpp->SumError += Rpp->LastError;

    /* 积分限幅 */
    Rpp->SumError = (Rpp->SumError > 10000) ? 10000 : ((Rpp->SumError < -10000) ? -10000 : Rpp->SumError);

    rightTemp = Rpp->kp * Rpp->LastError + Rpp->ki * Rpp->SumError; // 比例项 + 积分项

    OutputValue->righWheel = vehicleSpeed->righWheel + rightTemp;

    return RT_EOK;
}
#endif  /* MOTOREN */

/*****************
运行状态检测

倾斜角度是否过大，已实现
是否被提起，待完善：左右电机转速大于30、方向相同、持续时间超过250ms，且车身角度不超过30度，则判断为悬空空转
******************/
static rt_int16_t RunningDetecte(float FinalAngle, VehicleSpeed *OutputValue)
{
    if (MAXIMUM_TILT_ANGLE < abs(FinalAngle - 90))
    {
        /* 电机停止输出 */
        OutputValue->leftWheel = 0;
        OutputValue->righWheel = 0;

        /* 清除速度环积分项 */
        VelocityHead.SumError = 0;
    }


    return RT_EOK;
}

/* 设置电机输出
 * Output Value:
 * -10,000 ~ 10,000
 */
static rt_int16_t setOutput(VehicleSpeed *OutputValue)
{
    /* Judge Direction */
    if (0 < OutputValue->leftWheel)
    {
        /* 左轮前进 */
        AIN1(0);AIN2(1);
    } else {
        /* 左轮后退 */
        AIN1(1);AIN2(0);
    }

    if (0 < OutputValue->righWheel)
    {
        /* 右轮前进 */
        BIN1(0);BIN2(1);
    } else {
        /* 右轮后退 */
        BIN1(1);BIN2(0);
    }

    /* Limit */
    if (PERIOD < OutputValue->leftWheel)
    {
        OutputValue->leftWheel = PERIOD;
    }
    else if (-PERIOD > OutputValue->leftWheel)
    {
        OutputValue->leftWheel = -PERIOD;
    }
    if (PERIOD < OutputValue->righWheel)
    {
        OutputValue->righWheel = PERIOD;
    }
    else if (-PERIOD > OutputValue->righWheel)
    {
        OutputValue->righWheel = -PERIOD;
    }

    /* Output PWM */
#if MOTORDEADZONE
//    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_LEFT, PERIOD, abs(OutputValue->leftWheel) + LiftDeadZone);
//    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_RIGHT, PERIOD, abs(OutputValue->righWheel) + RightDeadZone);
#else
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_LEFT, PERIOD, abs(OutputValue->leftWheel));
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_RIGHT, PERIOD, abs(OutputValue->righWheel));
#endif  /* MOTORDEADZONE */

    return RT_EOK;
}
