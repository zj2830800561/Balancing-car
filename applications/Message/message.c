#include <stdio.h>
#include <string.h>
#include "DebugConfig.h"
#include "message.h"
#include "mpu6050.h"
#include "encoder.h"
#include "ps2.h"


#define BYTE0(dwTemp)       ((dwTemp >> (8 * 0)) & 0xff)     				//取出int型变量的低字节
#define BYTE1(dwTemp)       ((dwTemp >> (8 * 1)) & 0xff)     	//取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       ((dwTemp >> (8 * 2)) & 0xff)
#define BYTE3(dwTemp)       ((dwTemp >> (8 * 3)) & 0xff)

double rx_data[3] = {0}; // 存储解析后的数据
double rx_data_Bck[3] = {0}; // 存储解析后的数据

/* 其他线程的数据和各自的信号量 */
/* IMU */
extern rt_sem_t MPU_DATA_sem;
extern struct mpu6050_data MPU_Data;
extern rt_sem_t Filter_DATA_sem;
extern KalmanFilter filter;
/* Pulse Encoder */
extern rt_sem_t Speed_DATA_sem;
extern VehicleSpeed vehicleSpeed;
/* PS2 */
extern rt_sem_t Direction_DATA_sem;
extern rt_err_t PS2ConnectedFlag;
extern PS2_Light_Mode_t PS2WorkMode;
extern PS2_keys_data_t PS2_Keys;

/* Serial Print */
void Print_thread_entry(void)
{
#if (PRINTWITHOUTTOOL && (PRINTMPUDATA || PRINTFILTERANGLE || PRINTENCODER || PRINTPS2KEY))
    rt_err_t ret = RT_ERROR;
#endif

    for (;;)
    {
#if PRINTWITHOUTTOOL
#if PRINTMPUDATA
       ret = rt_sem_take(MPU_DATA_sem, 0x05);
       /*  Print: Accl_pitch, Gyro_pitch, Gyro_gyroX */
       if(RT_EOK == ret)
       {
           rt_kprintf("Accl_pitch: %d\r\n", (rt_uint16_t)MPU_Data.accl_pitch);
           rt_kprintf("Gyro_pitch: %d\r\n", (rt_uint16_t)MPU_Data.gyro_pitch);
           rt_kprintf("Gyro_gyroX: %d\r\n\r\n", (rt_int16_t)MPU_Data.gyrox);

           rt_sem_release(MPU_DATA_sem);
       }
#endif	/* PRINTMPUDATA */

#if PRINTFILTERANGLE
        ret = rt_sem_take(Filter_DATA_sem, 0x05);
        /*  Print: Angle_Final */
        if(RT_EOK == ret)
        {
            rt_kprintf("Filter Angle_Final: %d\r\n\r\n", (rt_int16_t)filter.Angle_Final);
            rt_sem_release(Filter_DATA_sem);
        }
#endif	/* PRINTFILTERANGLE */

#if PRINTENCODER
       ret = rt_sem_take(Speed_DATA_sem, 0x05);
       /*  Print: Vehicle_Speed */
       if(RT_EOK == ret)
       {
           rt_kprintf("Encoder Vehicle_Speed Left: %d\r\n", (rt_int16_t)vehicleSpeed.leftWheel);
           rt_kprintf("Encoder Vehicle_Speed Right: %d\r\n\r\n", (rt_int16_t)vehicleSpeed.righWheel);
           rt_sem_release(Speed_DATA_sem);
       }
#endif	/* PRINTENCODER */

#if PRINTPS2KEY
       ret = rt_sem_take(Direction_DATA_sem, 0x05);
       /*  Print: PS2 Key */
       if(RT_EOK == ret)
       {
           /* 手柄是否连接成功 */
           rt_kprintf("PS2ConnectedFlag: %s\r\n", (rt_int16_t)PS2ConnectedFlag == RT_EOK ? "Connected" : "Not Connect!");
           /* 红绿灯模式 */
           rt_kprintf("PS2WorkMode: %s\r\n", (rt_int16_t)PS2WorkMode == PS2_LIGHT_MODE_RED ? "RED_MODE" : PS2WorkMode == PS2_LIGHT_MODE_NO_CONNECT ? "Not Connect" : "GREEN_MODE");
            /* 按键状态 */
           if(PS2_Keys.key_value[PS2_KEY_L1] == 0){
               rt_kprintf("L1 is pressed\r\n\r\n");
           }
           if(PS2_Keys.key_value[PS2_KEY_L2] == 0){
               rt_kprintf("L2 is pressed\r\n\r\n");
           }
           if(PS2_Keys.key_value[PS2_KEY_R1] == 0){
               rt_kprintf("R1 is pressed\r\n\r\n");
           }
           if(PS2_Keys.key_value[PS2_KEY_R2] == 0){
               rt_kprintf("R2 is pressed\r\n\r\n");
           }
           rt_sem_release(Direction_DATA_sem);
       }
#endif	/* PRINTPS2KEY */

        rt_thread_mdelay(500);
#else
        AngleSend(MPU_Data.accl_pitch, MPU_Data.gyro_pitch, filter.Angle_Final);
        SpeedSend(0, vehicleSpeed.leftWheel, vehicleSpeed.righWheel);

        rt_thread_mdelay(50);
#endif	/* PRINTWITHOUTTOOL */
    }
}

/*
 *	角度发送函数
 *	将MPU解析的角度和滤波后的角度发送到匿名上位机中显示波形
 *	帧ID：0xF1
*/
void AngleSend(rt_uint16_t AcclAngle, rt_uint16_t GyroAngle, rt_uint16_t FilterAngle)
{
	unsigned char _cnt = 0;
	unsigned char i;
	unsigned char sumcheck = 0;
	unsigned char addcheck = 0;
	unsigned char Data_to_Send[12];
	
	Data_to_Send[_cnt++] = 0xAA;	/* HEAD */
	Data_to_Send[_cnt++] = 0xFF;	/* D_ADDR */
	Data_to_Send[_cnt++] = 0xF1;	/* ID */
	
	Data_to_Send[_cnt++] = 6;	/* LEN */
	/* DATA */
	Data_to_Send[_cnt++] = BYTE0(AcclAngle);	
	Data_to_Send[_cnt++] = BYTE1(AcclAngle);	
	Data_to_Send[_cnt++] = BYTE0(GyroAngle);	
	Data_to_Send[_cnt++] = BYTE1(GyroAngle);	
	Data_to_Send[_cnt++] = BYTE0(FilterAngle);	
	Data_to_Send[_cnt++] = BYTE1(FilterAngle);	
	
	for(i = 0; i < _cnt; i++)
	{
		sumcheck += Data_to_Send[i];
		addcheck += sumcheck;
	}
	
	Data_to_Send[_cnt++]=sumcheck;	/* SC(Sum check) */
	Data_to_Send[_cnt++]=addcheck;	/* AC(Add check) */
	
	for(i = 0; i < _cnt; i++)
	{
		/* HAL库函数 */
		/* HAL_UART_Transmit(&huart1, Data_to_Send, 12, 0xFF); */

		/* RT-Thread打印函数 */
		rt_kprintf("%d", Data_to_Send[i]);

		/* RT-Thread串口设备发送函数 */
		/* rt_device_write(uart1, 0, Data_to_Send, 12); */

	}
}


/*
 *	速度发送函数
 *	将电机运行的目标速度和现在的速度发送到匿名上位机中显示波形
 *	帧ID：0xF2
*/
void SpeedSend(int target, int Lcurrent, int Rcurrent)
{
	unsigned char _cnt = 0;
	unsigned char i;
	unsigned char sumcheck = 0;
	unsigned char addcheck = 0;
	unsigned char Data_to_Send[12];
	
	Data_to_Send[_cnt++] = 0xAA;	/* HEAD */
	Data_to_Send[_cnt++] = 0xFF;	/* D_ADDR */
	Data_to_Send[_cnt++] = 0xF2;	/* ID */
	
	Data_to_Send[_cnt++] = 6;	/* LEN */
	/* DATA */
	Data_to_Send[_cnt++] = BYTE0(target);	
	Data_to_Send[_cnt++] = BYTE1(target);	
	Data_to_Send[_cnt++] = BYTE0(Lcurrent);	
	Data_to_Send[_cnt++] = BYTE2(Lcurrent);	
	Data_to_Send[_cnt++] = BYTE0(Rcurrent);	
	Data_to_Send[_cnt++] = BYTE1(Rcurrent);	
	
	for(i = 0; i < _cnt; i++)
	{
		sumcheck += Data_to_Send[i];
		addcheck += sumcheck;
	}
	
	Data_to_Send[_cnt++]=sumcheck;	/* SC(Sum check) */
	Data_to_Send[_cnt++]=addcheck;	/* AC(Add check) */
	
	for(i = 0; i < _cnt; i++)
	{
		/*HAL_UART_Transmit(&huart1, Data_to_Send, 12, 0xFF);*/
		rt_kprintf("%02X", Data_to_Send[i]);
	}
}
