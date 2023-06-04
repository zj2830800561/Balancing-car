/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-23     Jie          the first version
 */

#ifndef __PS2_H_
#define __PS2_H_

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"


// PIN
#define PS2_CS_PIN      GET_PIN(B, 0)
#define PS2_SCK_PIN     GET_PIN(B, 1)
#define PS2_DI_PIN      GET_PIN(B, 10)
#define PS2_DO_PIN      GET_PIN(B, 11)
// COMMAND
#define PS2_CMD_VIBRATE     1

// MODE
#define PS2_NO_MODE         0
#define PS2_GREEN_MODE      0x41
#define PS2_RED_MODE        0x73

// KEY
#define PS2_BTN_SELECT      (1 << 0)
#define PS2_BTN_L3          (1 << 1)
#define PS2_BTN_R3          (1 << 2)
#define PS2_BTN_START       (1 << 3)
#define PS2_BTN_UP          (1 << 4)
#define PS2_BTN_RIGHT       (1 << 5)
#define PS2_BTN_DOWN        (1 << 6)
#define PS2_BTN_LEFT        (1 << 7)
#define PS2_BTN_L2          (1 << 8)
#define PS2_BTN_R2          (1 << 9)
#define PS2_BTN_L1          (1 << 10)
#define PS2_BTN_R1          (1 << 11)
#define PS2_BTN_TRIANGLE    (1 << 12)
#define PS2_BTN_CICLE       (1 << 13)
#define PS2_BTN_FORK        (1 << 14)
#define PS2_BTN_SQUARE      (1 << 15)

#define PS2_KEY_COUNTS       20

#define KEEP_TIME()                 _delay_us(8);

typedef enum
{
    PS2_KEY_SELECT      = 0,
    PS2_KEY_L3          = 1,
    PS2_KEY_R3          = 2,
    PS2_KEY_START       = 3,
    PS2_KEY_PAD_UP      = 4,
    PS2_KEY_PAD_RIGHT   = 5,
    PS2_KEY_PAD_DOWN    = 6,
    PS2_KEY_PAD_LEFT    = 7,
    PS2_KEY_L2          = 8,
    PS2_KEY_R2          = 9,
    PS2_KEY_L1          = 10,
    PS2_KEY_R1          = 11,
    PS2_KEY_GREEN       = 12,   /* UP       △  */
    PS2_KEY_RED         = 13,   /* RIGHT    〇  */
    PS2_KEY_BLUE        = 14,   /* DOWN     ×   */
    PS2_KEY_PINK        = 15,   /* LEFT     口  */
    PS2_KEY_ROCKER_LX   = 16,
    PS2_KEY_ROCKER_LY   = 17,
    PS2_KEY_ROCKER_RX   = 18,
    PS2_KEY_ROCKER_RY   = 19,
} ps2_key_index;

typedef enum
{
    PS2_LIGHT_MODE_NO_CONNECT,
    PS2_LIGHT_MODE_RED,
    PS2_LIGHT_MODE_GREEN,
} PS2_Light_Mode_t;

typedef struct
{ 
    uint8_t     key_value[PS2_KEY_COUNTS];  /* Array for storing keystroke values:key_value[ps2_key_index] */
} PS2_keys_data_t;


typedef struct 
{
    uint16_t button;            // 16个按键
    uint8_t left_stick_x;       // 左摇杆模拟量
    uint8_t left_stick_y;       // 左摇杆模拟量
    uint8_t right_stick_x;      // 右摇杆模拟量
    uint8_t right_stick_y;      // 右摇杆模拟量
} ps2_ctrl_data_t;

struct ps2_table
{
    int ps2_cmd;
    int standard_cmd;
};

void Remote_thread_entry(void);

PS2_Light_Mode_t PS2_ReadLight(void);
rt_err_t PS2_Get_Keys(PS2_keys_data_t *KeyBuffer);

#endif	/* __PS2_H_ */
