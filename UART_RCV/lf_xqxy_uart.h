/*
 * @Descripttion: 
 * @version: 
 * @Author: Newt
 * @Date: 2023-01-10 10:11:32
 * @LastEditors: Newt
 * @LastEditTime: 2023-01-31 10:29:32
 */
/*
* Copyright (c) 2021 Leapfive.
*
* This file is part of
* *** Leapfive Software Dev Kit ***
* (see www.leapfive.com).
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* 3. Neither the name of Leapfive Lab nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _LF_XQXY_UART_H_
#define _LF_XQXY_UART_H_

#include <stdint.h>

typedef struct
{
    uint8_t ptl_ver;     //计量模块协议版本
    uint8_t lock_ver;    //记录模块加密版本
    uint8_t sw_ver;      //计量模块程序版本
    uint8_t ted;         //保留
    uint8_t fac_num;     //计量模块厂家编号
    uint8_t xh_num;      //计量模块型号编号
    uint8_t batch_num;   //计量模块批次编号
    uint8_t uni_ide[5];  //唯一标识
}lf_mod_info;

#define UART_FRAME_HEAD_LEN         4                      //有效帧头长度，包括帧长和帧长之前的数据长度
#define UART_FRAME_TYPE_INDEX       1                       //帧类型数据位偏移
#define UART_FRAME_LEN_INDEX        3                       //帧长数据位偏移
#define UART_OTHER_DATA_LEN         5                      //其余数据总长(有效数据除外)
#define RCV_DATA_LEN                150                     //串口接收数据大小
#define SEND_DATA_LEN               150                     //串口接收数据大小
#define UART_BAUD                   9600                    //串口波特率
#define NONE                        0x00                    //无
#define AWAKEN_HEAD                 0xFE                    //唤醒前导字节
#define AWAKEN_READ                 0x01                    //读操作
#define AWAKEN_WRITE                0x02                    //写操作
#define AWAKEN_UPLOAD               0x03                    //上传
#define AWAKEN_CTRL                 0x04                    //控制类
#define ID_DATA_LEN                 12                      //ID信息长度
#define MAX_VAL_LEN                 11                      //有效数据最大长度

#define TOTAL_ELEC                  0x01                    //总有功电量
#define TOTAL_AELEC                 0x11                    //A相有功电量
#define TOTAL_BELEC                 0x21                    //B相有功电量
#define TOTAL_CELEC                 0x31                    //C相有功电量
#define A_VOL                       0x12                    //A相电压
#define B_VOL                       0x22                    //B相电压
#define C_VOL                       0x32                    //C相电压
#define A_CUR                       0x13                    //A相电流
#define B_CUR                       0x23                    //B相电流
#define C_CUR                       0x33                    //C相电流
#define FREQ                        0x04                    //频率
#define TOTAL_ACTPOW                0x05                    //总有功功率
#define A_ACTPOW                    0x15                    //A相有功功率
#define B_ACTPOW                    0x25                    //B相有功功率
#define C_ACTPOW                    0x35                    //C相有功功率
#define TOTAL_REAPOW                0x06                    //总无功功率
#define A_REAPOW                    0x16                    //A相无功功率
#define B_REAPOW                    0x26                    //B相无功功率
#define C_REAPOW                    0x36                    //C相无功功率
#define TOTAL_APPAPOW               0x07                    //总视在功率
#define A_APPAPOW                   0x17                    //A相视在功率
#define B_APPAPOW                   0x27                    //B相视在功率
#define C_APPAPOW                   0x37                    //C相视在功率
#define TOTAL_PFC                   0x08                    //总功率因数
#define A_PFC                       0x18                    //A相功率因数
#define B_PFC                       0x28                    //B相功率因数
#define C_PFC                       0x38                    //C相功率因数
#define TEMP                        0x09                    //温度
#define TIME                        0x43                    //日期时间
#define ELEC_FIND                   0x66                    //用电数据查询
#define OPPO_FIND                   0x77                    //数据反向查询
#define ALARM_STATE                 0x88                    //告警状态字
#define ID__NUM                     0x41                    //ID号
#define SPE_ID                      0x42                    //特殊帧ID号
#define CLEAR_POWER                  0xF9                    //清电量

#define VOLT_MAX                    275                     //过压阈值
#define VOLT_MIN                    160                     //欠压阈值
#define CURRENT_MAX                 1                       //过流阈值

/* 设置ID信息 */
uint8_t lf_uart1_set_id(lf_mod_info id_info);
/* 启动串口任务 */
int lf_uart1_task_init(void);


#endif