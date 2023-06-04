/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-23     Jie          the first version
 */

#include <rtthread.h>
#include <ps2.h>
#include <DebugConfig.h>

#define DBG_SECTION_NAME            "ps2"
#define DBG_LEVEL                   DBG_LOG
#include <rtdbg.h>

#define THREAD_DELAY_TIME           10
#define KEEP_TIME()                 _delay_us(8);

static ps2_ctrl_data_t  ctrl_data    =  {0};
static rt_err_t PS2_PIN_Init(void);
static rt_err_t PS2_Init(void);

/* 摇杆连接状态，供外部模块获取 */
rt_err_t PS2ConnectedFlag = RT_ERROR;
/* 摇杆控制模式，供外部模块获取 */
PS2_Light_Mode_t PS2WorkMode = PS2_LIGHT_MODE_NO_CONNECT;
/* 摇杆模拟量中间值，供外部模块获取 */
rt_uint8_t lx_rocker_offset = 128;
rt_uint8_t ly_rocker_offset = 127;
rt_uint8_t rx_rocker_offset = 128;
rt_uint8_t ry_rocker_offset = 127;
/* 按键状态，供外部模块获取 */
PS2_keys_data_t PS2_Keys = {0};

/* 按键数据信号量
 * 持续读取红绿运行模式会导致手柄重启
 * 如需读取运行模式需要改变策略
 * 绿灯模式下摇杆模拟量全为255
 * 以此判断运行模式 */
extern rt_sem_t Direction_DATA_sem;

void Remote_thread_entry(void)
{
    rt_err_t ret = RT_ERROR;
    PS2_Light_Mode_t mode = PS2_LIGHT_MODE_NO_CONNECT;

    while(RT_ERROR == PS2_Init());

    for(;;)
    {

        /* 如果断开连接 */
        while(PS2_LIGHT_MODE_NO_CONNECT == mode)
        {
            rt_kprintf("waiting for connect PS2!\r\n");
            ret = rt_sem_take(Direction_DATA_sem, 0x05);
            if(RT_EOK == ret)
            {
                PS2ConnectedFlag = RT_ERROR;
                rt_sem_release(Direction_DATA_sem);
            }

            rt_thread_mdelay(1000);

            mode = PS2_ReadLight();
        }

        /* 读取按键 */
        ret = rt_sem_take(Direction_DATA_sem, 0x05);
        if(RT_EOK == ret)
        {
            PS2ConnectedFlag = RT_EOK;

            PS2WorkMode = mode;

            PS2_Get_Keys(&PS2_Keys);

            rt_sem_release(Direction_DATA_sem);
        }

        /* Control_Task T: 10Hz */
        rt_thread_mdelay(100);
    }
}

static void CS_High(void)
{
    rt_pin_write(PS2_CS_PIN, PIN_HIGH);
}
static void CS_Low(void)
{
    rt_pin_write(PS2_CS_PIN, PIN_LOW);
}
static void CLK_High(void)
{
    rt_pin_write(PS2_SCK_PIN, PIN_HIGH);
}
static void CLK_Low(void)
{
    rt_pin_write(PS2_SCK_PIN, PIN_LOW);
}
static void DO_High(void)
{
    rt_pin_write(PS2_DO_PIN, PIN_HIGH);
}
static void DO_Low(void)
{
    rt_pin_write(PS2_DO_PIN, PIN_LOW);
}
static int Read_DI(void)
{
    return rt_pin_read(PS2_DI_PIN);
}

/* 简单延时 */
static void _delay_us(uint16_t us)
{
    for (int i = 0; i < us; i++)
    {
        for (int j = 0; j < 0x1F;)
            j++;
    }
}

/* 主机发送命令 */
static rt_uint8_t PS2_Cmd(rt_uint8_t CMD)
{
    rt_uint8_t temp = 0;

    for (uint16_t i = 0x01; i < 0x0100; i <<= 1)
    {
        if (i & CMD)
            DO_High(); 
        else
            DO_Low();

        CLK_High();
        KEEP_TIME();
        CLK_Low();
        if (Read_DI())
            temp = i | temp;
        KEEP_TIME();
        CLK_High();
    }
    /* KEEP_TIME(); */
    
    return temp;
}

/* 判断是否为红灯模式 */
PS2_Light_Mode_t PS2_ReadLight(void)
{
    rt_uint8_t light = 0;
    PS2_Light_Mode_t ret = PS2_LIGHT_MODE_NO_CONNECT;

	CS_Low();
	PS2_Cmd(0x01);              /* 开始命令 */
	light = PS2_Cmd(0x42);      /* 请求数据 */
	CS_High();

	if(PS2_RED_MODE == light)
        ret = PS2_LIGHT_MODE_RED;
	else if(PS2_GREEN_MODE == light)
        ret = PS2_LIGHT_MODE_GREEN;

    return ret;
}

/* 协议发送过程 */
static void transfer(const rt_uint8_t *pb_send, rt_uint8_t *pb_recv, rt_uint8_t len)
{
    CS_Low();
    KEEP_TIME();
    for (rt_uint8_t i = 0; i < len; i++)
    {
        pb_recv[i] = PS2_Cmd(pb_send[i]);
    }
    CS_High();
    KEEP_TIME();
}

/* 获取全部按键状态 */
static int PS2_Scan(ps2_ctrl_data_t *pData)
{
    rt_uint8_t temp[9] = {0};

    /* 主机需要发送的命令 */
    temp[0] = 0x01;
    temp[1] = 0x42;

    /* 发送过程 */
    transfer(temp, temp, 9);
    
    /* PS2发送0x5A，表示发送了按键数据（其实ID会先于0x5A发送，不受影响） */
    if (temp[2] == 0x5A)
    {
        /* 返回按键数据 */
        pData->button = temp[3] | (temp[4] << 8);  /* 16个独立按键 */
        pData->right_stick_x   = temp[5];          /* 2个摇杆的模拟值 */
        pData->right_stick_y   = temp[6];
        pData->left_stick_x    = temp[7];
        pData->left_stick_y    = temp[8];
    }
    else
    {
        rt_memcpy(pData, 0, sizeof(pData));
    }

    return RT_EOK;
}

rt_err_t PS2_Get_Keys(PS2_keys_data_t *KeyBuffer)
{
    if (RT_NULL == KeyBuffer)
        return -RT_EINVAL;
    
    PS2_Scan(&ctrl_data);

    /* 解析按键值，存储到数组中，该数组可供外部获取 */
    for (int i = 0; i < 16; i++)
    {
        if (ctrl_data.button & (0x01 << i))
        {
            KeyBuffer->key_value[i] = 1;
        }
        else
        {
            KeyBuffer->key_value[i] = 0;
        }
    }
    KeyBuffer->key_value[16] = ctrl_data.left_stick_x;
    KeyBuffer->key_value[17] = ctrl_data.left_stick_y;
    KeyBuffer->key_value[18] = ctrl_data.right_stick_x;
    KeyBuffer->key_value[19] = ctrl_data.right_stick_y;
    
    return RT_EOK;
}

/* PS2设备初始化 */
static rt_err_t PS2_Init(void)
{
    /* 引脚初始化 */
    PS2_PIN_Init();

    /* 当前未连接 */
    while(PS2_LIGHT_MODE_NO_CONNECT == PS2_ReadLight())
    {
        /*rt_kprintf("waiting for connect PS2!\r\n");*/
        rt_thread_mdelay(1000);
    }

#if 0
     等待红灯模式
    while(PS2_LIGHT_MODE_RED != PS2_ReadLight())
    {
        rt_kprintf("waiting for change to red mode!\r\n");
        rt_thread_mdelay(1000);
    }

     获取摇杆偏移量
    PS2_Get_Keys(&PS2_Keys);

     获取摇杆中间值
  第一次的数据非常不准确，于是直接按照静止状态下的偏移值为参考值，不再重新计算偏移量
    lx_rocker_offset = PS2_Keys.key_value[PS2_KEY_ROCKER_LX];
    ly_rocker_offset = PS2_Keys.key_value[PS2_KEY_ROCKER_LY];
    rx_rocker_offset = PS2_Keys.key_value[PS2_KEY_ROCKER_RX];
    ry_rocker_offset = PS2_Keys.key_value[PS2_KEY_ROCKER_RY];
#endif  /* 0 */

    rt_thread_mdelay(THREAD_DELAY_TIME);

    return RT_EOK;
}

/* PS2使用的引脚初始化 */
static rt_err_t PS2_PIN_Init(void)
{
    rt_pin_mode(PS2_CS_PIN,  PIN_MODE_OUTPUT);
    rt_pin_mode(PS2_SCK_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(PS2_DO_PIN,  PIN_MODE_OUTPUT);
    rt_pin_mode(PS2_DI_PIN,  PIN_MODE_INPUT);
    
    CS_High();
    CLK_High();

    return RT_EOK;
}


/* Drive Function Test */
#if PS2TASKCMD
static int PS2_test(void)
{
    PS2_Init();
    rt_kprintf("lx: %d\tly: %d\t\trx: %d\try: %d\r\n",
    lx_rocker_offset, ly_rocker_offset, rx_rocker_offset, ry_rocker_offset);

    for(;;){
        1.通讯是否正常 √
        rt_kprintf("PS2 Light Mode: %d\r\n\r\n", PS2_ReadLight());
        2.获取按键 √
        PS2_Get_Keys(&PS2_Keys);
        for (int var = 0; var < 20; ++var) {
            rt_kprintf("%d ", PS2_Keys.key_value[var]);
        }
        rt_kprintf("\r\n\r\n");

        rt_thread_mdelay(300);
    }
    
    return RT_EOK;
}
MSH_CMD_EXPORT(PS2_test, ps2 device test);
#endif  /* PS2TASKCMD */
