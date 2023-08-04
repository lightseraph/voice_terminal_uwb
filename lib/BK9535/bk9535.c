#define __BK9531_ProC__

#include "bk9535.h"
#include "syscall.h"
#include <stdlib.h>

// TX初始化表格。
unsigned char tx_reg_val[37][5] =
    {
        {0x00, 0x1E, 0x44, 0x0C, 0x88}, // REG00
        {0x01, 0x04, 0xFF, 0x00, 0x56}, // REG01
        {0x02, 0x89, 0x5C, 0x02, 0x9B}, // REG02
        {0x03, 0xEF, 0x24, 0x04, 0x00}, // REG03
        {0x04, 0x5D, 0x88, 0x00, 0x44}, // REG04
        {0x05, 0x00, 0x28, 0x03, 0x80}, // REG05
        {0x06, 0x59, 0x00, 0x78, 0x80}, // REG06
        {0x07, 0x1E, 0x40, 0x00, 0x00}, // REG07
#if 0
    0x08, 0x00, 0x08, 0x01, 0x00,   //REG08
#else
        {0x08, 0xAA, 0x08, 0x01, 0x00}, // REG08             2022-06-02 根据要求修改
#endif
        {0x09, 0x00, 0x00, 0x81, 0x03}, // REG09
        {0x0A, 0x72, 0xAB, 0x21, 0x1D}, // REG0A     发射功率
        {0x0B, 0x00, 0x00, 0xC0, 0x05}, // REG0B
        {0x0C, 0x00, 0x00, 0x02, 0xDE}, // REG0C
        {0x0D, 0x3A, 0x98, 0x00, 0x00}, // REG0D
        {0x30, 0x28, 0x28, 0x28, 0x28}, // REG30
        {0x31, 0xD1, 0x00, 0x00, 0x28}, // REG31
        {0x32, 0x10, 0x06, 0x00, 0x64}, // REG32
        {0x33, 0x48, 0x80, 0x8D, 0x82}, // REG33
        {0x34, 0x0B, 0x02, 0x11, 0x08}, // REG34     增益18dB
        {0x35, 0x70, 0x50, 0x00, 0x80}, // REG35
        {0x36, 0x0F, 0x80, 0x1E, 0x04}, // REG36
        {0x37, 0x00, 0x00, 0x00, 0x00}, // REG37
        {0x38, 0x00, 0x00, 0x00, 0x00}, // REG38
        {0x39, 0x03, 0xD7, 0xD5, 0xF7}, // REG39
        {0x3A, 0xE0, 0x25, 0x00, 0x74}, // REG3A
        {0x3B, 0x85, 0x25, 0x00, 0x3A}, // REG3B
        {0x3C, 0x77, 0x01, 0x00, 0x3B}, // REG3C
        {0x3D, 0x95, 0x23, 0x00, 0x3C}, // REG3D
        {0x3E, 0x00, 0xF8, 0x07, 0xC0}, // REG3E
        {0x3F, 0x80, 0x0F, 0x00, 0x00}, // REG3F
        {0x70, 0x00, 0x00, 0x95, 0x35}, // REG70
        {0x71, 0x21, 0x82, 0x08, 0x10}, // REG71
        {0x72, 0x00, 0x5A, 0x12, 0x1A}, // REG72
        {0x73, 0x00, 0x00, 0x00, 0x00}, // REG73
        {0x77, 0x00, 0x4A, 0x30, 0x53}, // REG77
        {0x78, 0x00, 0x00, 0x00, 0x08}, // REG78
        {0xFF, 0x00, 0x00, 0x00, 0x00}, // 表格结束
};

// TX初始化函数
u8 BK_Init(void)
{
    unsigned char retry;
    struct InitData
    {
        unsigned char reg;        // 寄存器地址
        unsigned char reg_val[4]; // 寄存器设置值
    };
    struct InitData *reg_init;

    // CE = 1;
    // I2C_Init();
    retry = TX_INIT_RETRY_TIMES;
    while (retry--)
    {
        delay_ms(20);
        if (!TX_I2C_Read(0x70, reg_val))
            continue;
        if (reg_val[3] == TX_Chip_ID)
            break;
    }
    if (reg_val[3] != TX_Chip_ID)
        return 0;
    //
    reg_init = (struct InitData *)tx_reg_val;
    while (reg_init->reg != 0xFF)
    {
        TX_I2C_Write(reg_init->reg, reg_init->reg_val);
        reg_init++;
    }
    SamplingRateSel(DEF_WORK_MODE); // 设置工作采样率
    rTriggerValue = 0x04;
    TX_Reset_Chip();
    return 1;
}

void TX_Reset_Chip(void)
{
    TX_I2C_Read(0x3F, reg_val);
    reg_val[1] &= ~0x08; //
    TX_I2C_Write(0x3F, reg_val);
    delay_ms(1);
    reg_val[1] |= 0x08; //
    TX_I2C_Write(0x3F, reg_val);
}

void TX_SingleWave_Start(void) // 0x20 for BK9520, 0x3F for BK9526/32
{
    TX_I2C_Read(0x3F, reg_val);
    reg_val[0] |= 0x40; // REG20[1:0]=2 for test_pat and [7]=tx_en=1
    reg_val[0] |= 0x80;
    TX_I2C_Write(0x3F, reg_val);

    reg_val[0] &= ~0x80; // REG20[7]=tx_en=0
    TX_I2C_Write(0x3F, reg_val);

    reg_val[1] &= ~0x08; // soft rst to 0
    TX_I2C_Write(0x3F, reg_val);
}

void TX_SingleWave_Stop(void) // refer to above
{
    TX_I2C_Read(0x3f, reg_val);
    reg_val[0] &= ~0x40; // REG20[1:0]=0 for test_pat and [7]=tx_en=0
    reg_val[0] &= ~0x80;
    TX_I2C_Write(0x3f, reg_val);

    reg_val[0] |= 0x80; // REG20[7]=tx_en=1
    TX_I2C_Write(0x3f, reg_val);

    reg_val[1] |= 0x08; // soft rst to 1
    TX_I2C_Write(0x3F, reg_val);
}

void TX_PN9_Start(void)
{
    TX_I2C_Read(0x3F, reg_val);
    reg_val[0] |= 0x80; // tx_en = 1, test mode = 1
    reg_val[0] |= 0x40;
    TX_I2C_Write(0x3F, reg_val);
    reg_val[0] &= ~0x1F; // test pattern [28:20] = 0, all set to 0
    reg_val[1] &= ~0xF0;
    TX_I2C_Write(0x3F, reg_val);
}

void TX_PN9_Stop(void)
{
    TX_I2C_Read(0x3F, reg_val);
    reg_val[0] |= 0x80; // tx_en = 1, test mode = 0
    reg_val[0] &= ~0x40;
    TX_I2C_Write(0x3F, reg_val);
}

// 切频函数
void TX_TuneFreq(unsigned long khz)
{
    unsigned long freqdat;
    unsigned char tmp[4] = {0};
    unsigned char status;

    TX_I2C_Read(0x06, tmp);
    status = tmp[2];
    tmp[2] = 0xB8;
    TX_I2C_Write(0x06, tmp);

    if (khz < 500000)
    {
        return; // 暂时不支持V段频点
    }
    else if (khz <= 710000)
    {
        freqdat = (khz * 6 * 341) + ((khz * 6 + 2) / 3);
        rTriggerValue = 0x84;
    }
    else if (khz <= 1176000)
    {
        freqdat = (khz * 4 * 341) + ((khz * 4 + 2) / 3);
        rTriggerValue = 0x04;
    }
    else
    {
        khz = 1176000;
        freqdat = (khz * 4 * 341) + ((khz * 4 + 2) / 3);
        rTriggerValue = 0x04;
    }
    //
    TX_I2C_Read(0x0C, reg_val);
    if (khz > 980000)
    {
        reg_val[3] &= ~(1 << 4);
    }
    else
    {
        reg_val[3] |= (1 << 4);
    }
    TX_I2C_Write(0x0C, reg_val);
    //
    TX_I2C_WriteReg(0x0D, freqdat);
    TX_Trigger();
    //
    tmp[2] = status;
    TX_I2C_Write(0x06, tmp); // 恢复REG06
}

// 切到索引指定的频点
void SwitchFreqByIndex(unsigned char index)
{
    if (CHA == rWorkChannel)
    {
        TX_TuneFreq(FreqTableA[index] * 100ul);
    }
    else
    {
        TX_TuneFreq(FreqTableB[index] * 100ul);
    }
}

// 切到上一个频点
void SwitchNextFreq(void)
{
    if (USER_DATA.rUserFreqIndex == FREQ_NUM)
    {
        USER_DATA.rUserFreqIndex = 1;
    }
    else
    {
        USER_DATA.rUserFreqIndex++;
    }
    SwitchFreqByIndex(USER_DATA.rUserFreqIndex);
}

// 切到上一个频点
void SwitchPrevFreq(void)
{
    if (USER_DATA.rUserFreqIndex == 1)
    {
        USER_DATA.rUserFreqIndex = FREQ_NUM;
    }
    else
    {
        USER_DATA.rUserFreqIndex--;
    }
    SwitchFreqByIndex(USER_DATA.rUserFreqIndex);
}

// 写寄存器
void TX_I2C_WriteReg(unsigned char reg, unsigned long dat)
{
    reg_val[3] = (dat & 0xff);
    dat >>= 8;
    reg_val[2] = (dat & 0xff);
    dat >>= 8;
    reg_val[1] = (dat & 0xff);
    dat >>= 8;
    reg_val[0] = (dat & 0xff);
    TX_I2C_Write(reg, reg_val);
}

// 读寄存器
unsigned long TX_I2C_ReadReg(unsigned char reg)
{
    unsigned long ret;

    TX_I2C_Read(reg, reg_val);
    ret = reg_val[0];
    ret <<= 8;
    ret |= reg_val[1];
    ret <<= 8;
    ret |= reg_val[2];
    ret <<= 8;
    ret |= reg_val[3];
    return ret;
}

// 写用户数据（数据通道）
void TX_WriteUserData(unsigned char dat)
{
    TX_I2C_Read(0x3A, reg_val);
    reg_val[3] = dat;
    TX_I2C_Write(0x3A, reg_val);
}

void TX_Trigger(void)
{
    unsigned char i;

    for (i = 0; i < TRIGGER_RETRY; i++)
    {
        reg_val[0] = 0xEF;
        reg_val[1] = 0xA4;
        reg_val[2] = rTriggerValue;
        reg_val[3] = 0x00;

        TX_I2C_Write(0x03, reg_val); // REG3[23] = 1,触发一次校准
        delay_ms(50);
        reg_val[1] = 0x24;
        TX_I2C_Write(0x03, reg_val); // REG3[23] = 0,恢复原值

        delay_ms(10);
        TX_I2C_Read(0x72, reg_val); // 检查是否校准成功
        if ((reg_val[1] & 0x40) && ((reg_val[2] & 0x01) == 0x00))
            return; // 校准完成并且PLL锁定
        delay_ms(50);
    }
}

void TX_Prevent_RF_UnLock(void)
{
    TX_I2C_Read(0x72, reg_val);
    if (reg_val[2] & 0x01)
        SwitchFreqByIndex(USER_DATA.rUserFreqIndex); // 检测到失锁需要重新设频点
}

// 音量设置
void Set_TX_Volume(unsigned char vol)
{
    unsigned char temp1;
    unsigned char temp2;

    if (vol > 95)
        vol = 95;

    temp1 = vol / 12;
    temp2 = vol % 12;
    temp2 = temp2 << 4;

    TX_I2C_Read(0x34, reg_val);
    reg_val[0] |= 0x08;                       // REG34[27] = 1,手动增益
    reg_val[1] &= ~0x08;                      // REG34[19] = 0,手动增益
    reg_val[0] = (reg_val[0] & 0xf8) | temp1; // REG34[26:24] = temp1
    reg_val[1] = (reg_val[1] & 0x0f) | temp2; // REG34[23:20] = temp2
    TX_I2C_Write(0x34, reg_val);
}

// 开音频指示（占用GPIO3）。
void EnableAudioIndicator(void)
{
    TX_I2C_Read(0x30, reg_val);
    reg_val[0] = 0x40;
    TX_I2C_Write(0x30, reg_val);
    //
    TX_I2C_Read(0x31, reg_val);
    reg_val[1] &= ~(7 << 1);
    TX_I2C_Write(0x31, reg_val);
}

u8 TX_I2C_Write(u8 reg, u8 *buf)
{
    unsigned char retry, i;

    if (reg <= 0x0b)
    {
        for (i = 0; i < 4; i++)
        {
            analog_reg_val[reg][i] = buf[i];
        }
    }
    //
    retry = TX_I2C_RETRY_TIMES;
    while (retry--)
    {
        IIC_Start();
        IIC_SendByte(CHIP_DEV_TX);
        if (IIC_Wait_Ack())
            continue;
        reg = reg << 1;
        IIC_SendByte(reg);
        if (IIC_Wait_Ack())
            continue;
        for (i = 0; i < 4; i++)
        {
            IIC_SendByte(buf[i]);
            IIC_Wait_Ack();
        }
        break;
    }
    IIC_Stop();
    if (retry == 0)
        return 0;
    else
        return 1;
}

u8 TX_I2C_Read(u8 reg, u8 *buf)
{
    unsigned char retry = 0, i;

    if (reg <= 0x0b)
    {
        for (i = 0; i < 4; i++)
        {
            buf[i] = analog_reg_val[reg][i];
        }
        return 1;
    }
    else
    {
        retry = TX_I2C_RETRY_TIMES;
        while (retry--)
        {
            IIC_Start();
            IIC_SendByte(CHIP_DEV_TX);
            if (IIC_Wait_Ack())
                continue;

            reg = reg << 1;
            reg |= 0x01;
            IIC_SendByte(reg);
            if (IIC_Wait_Ack())
                continue;

            for (i = 0; i < 3; i++)
            {
                buf[i] = IIC_ReadByte(1);
                // IIC_Ack();
            }
            buf[i] = IIC_ReadByte(0);
            // IIC_NAck();
            break;
        }
        IIC_Stop();
    }
    if (retry == 0)
        return 0;
    else
        return 1;
}

#if USE_I2S_AUDOI
// 开启I2S总线，并将音源切换到I2S音源。
void BK_Tx_I2SOpen(t_PCMCfg cfg)
{
    unsigned long tmp;

    tmp = TX_I2C_ReadReg(0x30);
    if (cfg.dat == PCM_SDA_I)
    {
        tmp &= 0x00ff0000; // GPIO2端口状态保持不变
        tmp |= 0x7D000000; // GPIO3作为DATA输入
    }
    else
    {
        tmp &= 0xff000000; // GPIO3端口状态保持不变
        tmp |= 0x00400000; // GPIO2作为DATA输出
    }
    if (cfg.mode == PCM_MASTER)
    {
        cfg.bclk = PCM_SCK_O; // 在主模式下BCLK和LRCK被强制作为输出
        cfg.lrck = PCM_SCK_O;
        tmp |= 0x00004040;
    }
    else
    {
        cfg.bclk = PCM_SCK_I; // 在从模式下BCLK和LRCK被强制作为输入
        cfg.lrck = PCM_SCK_I;
        tmp |= 0x00007D7D;
    }
    TX_I2C_WriteReg(0x30, tmp); // 设定GPIO模式
    tmp = TX_I2C_ReadReg(0x31);
    if (cfg.dat == PCM_SDA_I)
    {
        tmp &= 0xFFF1C0FF;
        tmp |= ((unsigned long)cfg.dat << 17);
    }
    else
    {
        tmp &= 0xFFFE00FF;
        tmp |= ((unsigned long)cfg.dat << 14);
    }
    tmp |= (((unsigned long)cfg.bclk << 11) | ((unsigned long)cfg.lrck << 8));
    //////////////////////////////////
#if 0 // 打开GPIO4的MCLK功能
    tmp &= 0xFFFFFF00;
    tmp |= 0x40; // GPIO4设为第二功能输出
    tmp &= ~(7ul << 20);
    tmp |= (3ul << 20); // 设为MCLK
#endif
    //////////////////////////////////
    TX_I2C_WriteReg(0x31, tmp); // GPIO第二功能选择
    tmp = TX_I2C_ReadReg(0x36);
    tmp &= 0xFE0000C1;                                    // 选择I2S工作协议
    tmp |= (((unsigned long)cfg.mode << 1) | (1ul << 0)); // 使能I2S，并选择工作模式。
    tmp |= 0x00009800;                                    // 默认24bits数据长度
    TX_I2C_WriteReg(0x36, tmp);
    TX_I2C_WriteReg(0x37, 0x000007C0); // 默认24.576MHz晶振，48K采样率
    // 设置声道
    tmp = TX_I2C_ReadReg(0x37);
    if (cfg.ch)
    {
        tmp |= (1ul << 13);
    }
    else
    {
        tmp &= ~(1ul << 13);
    }
    TX_I2C_WriteReg(0x37, tmp); // 选择I2S声道
    // 设置音源为I2S
    tmp = TX_I2C_ReadReg(0x39);
    tmp |= (1ul << 27);
    TX_I2C_WriteReg(0x39, tmp); // 选择I2S数字音源
}
#endif

t_WorkModeCfg WorkMode[] = {
    {(0x03 << 3), (0x00 << 6)}, // 12K
    {(0x02 << 3), (0x01 << 6)}, // 24K
    {(0x01 << 3), (0x02 << 6)}, // 32K
    {(0x00 << 3), (0x03 << 6)}, // 48K
};

// 设置工作采样率
void SamplingRateSel(e_WorkMode mode)
{
    // analog_reg_val[5][0] &= ~0x18;
    analog_reg_val[5][0] |= WorkMode[mode].reg5_28_27;
    TX_I2C_Write(0x05, (u8 *)analog_reg_val[5]);

    analog_reg_val[8][1] &= ~0x10;
    TX_I2C_Write(0x08, (u8 *)analog_reg_val[8]);

    TX_I2C_Read(0x31, reg_val);
    reg_val[0] &= ~0xc0;
    reg_val[0] |= WorkMode[mode].reg31_31_30;
    TX_I2C_Write(0x31, reg_val);
}

void Adjust_XTAL(u8 value)
{
    TX_I2C_Read(0x3A, reg_val);
}