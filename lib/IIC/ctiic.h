#ifndef __CT_IIC_H
#define __CT_IIC_H
// #include "sys.h"
#include "main.h"
//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
//  ALIENTEK STM32开发板
// 电容触摸屏-IIC 驱动代码
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 创建日期:2015/12/30
// 版本：V1.0
// 版权所有，盗版必究。
//  Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//  All rights reserved
//********************************************************************************
// 无
//////////////////////////////////////////////////////////////////////////////////

// IO方向设置
#define IIC_SDA_IN()                    \
    {                                   \
        GPIOA->ODR &= ~(3 << (12 * 2)); \
        GPIOA->ODR |= 0 << 12 * 2;      \
    } // PG7输入模式
#define IIC_SDA_OUT()                   \
    {                                   \
        GPIOA->ODR &= ~(3 << (12 * 2)); \
        GPIOA->ODR |= 1 << 12 * 2;      \
    } // PG7输出模式
// IO操作函数
#define IIC_SCL(n) (n ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)) // SCL
#define IIC_SDA(n) (n ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)) // SDA
#define IIC_READ_SDA HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)                                                                            // 输入SDA

// IIC所有操作函数
void IIC_Init(void);       // 初始化IIC的IO口
void IIC_Start(void);      // 发送IIC开始信号
void IIC_Stop(void);       // 发送IIC停止信号
void IIC_SendByte(u8 txd); // IIC发送一个字节
u8 IIC_ReadByte(u8 ack);   // IIC读取一个字节
u8 IIC_Wait_Ack(void);     // IIC等待ACK信号
void IIC_Ack(void);        // IIC发送ACK信号
void IIC_NAck(void);       // IIC不发送ACK信号
u8 IIC_SendByte_ack(u8 txd);

#endif