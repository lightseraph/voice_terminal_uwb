#ifndef __CONFIG_H__
#define __CONFIG_H__

typedef enum
{
    PCM_SLAVE = 0, // I2S从机模式
    PCM_MASTER,    // I2S主机模式
} e_PcmMode;

typedef enum
{
    PCM_SDA_O = 0, // PCM_SDA引脚设为输出
    PCM_SDA_I = 4, // PCM_SDA引脚设为输入
} e_PcmDataCfg;

typedef enum
{
    PCM_SCK_O = 0, // PCM_SCK引脚设为输出
    PCM_SCK_I = 4, // PCM_SCK引脚设为输入
} e_PcmBclkCfg;

typedef enum
{
    PCM_LRCK_O = 0, // PCM_LRCK引脚设为输出
    PCM_LRCK_I = 4, // PCM_LRCK引脚设为输入
} e_PcmLrckCfg;

typedef enum
{
    RIGHT_CHANNEL = 0, // 选择I2S右声道所为音源
    LEFT_CHANNEL,      // 选择I2S左声道所为音源
    CHANNEL_MONO,      // 单声道（未使用）
} e_PcmChCfg;

typedef struct
{ // I2S配置结构体
    e_PcmMode mode;
    e_PcmDataCfg dat;
    e_PcmBclkCfg bclk;
    e_PcmLrckCfg lrck;
    e_PcmChCfg ch;
} t_PCMCfg;

typedef enum
{ // 工作采样率
    WORK_MODE12K = 0,
    WORK_MODE24K,
    WORK_MODE32K,
    WORK_MODE48K,
    TOTEL_WORK_MODES,
} e_WorkMode;

typedef struct
{ // 工作采样率配置寄存器结构体
    unsigned char reg5_28_27;
    unsigned char reg31_31_30;
} t_WorkModeCfg;

typedef enum
{
    CHA = 0, // 使用A通道频点
    CHB,     // 使用B通道频点
} e_ChannleDef;

typedef union
{
    unsigned char byte[4];
    unsigned long dword;
} DWORD_DEF;

typedef struct
{
    DWORD_DEF UserId;             // 当前工作ID
    unsigned char rUserFreqIndex; // 频点索引
} USER_DATA_DEF;

#define DEF_WORK_MODE WORK_MODE48K // 工作模式（采样率）

#define DEF_PAIR_ID 0x12345678ul // 对频ID
#define DEF_USER_ID 0x00000000ul // 出厂默认ID

#define PAIR_FREQ_CHA 7700 // A通道对频频点
#define PAIR_FREQ_CHB 7490 // B通道对频频点

#define FREQ_NUM 4  // 工作频点数量
#define FREQ_STEP 3 // 工作频点间隔

#define START_FREQ_CHA 6400 // A通道起始频点
#define START_FREQ_CHB 7500 // B通道起始频点

#define USE_I2S_AUDOI 1          // 0 - 使用MIC作为音源, 1 - 使用I2S音源
#define USE_BK9524_IR_FUNCTION 0 // 0 - 关闭BK9524红外码输出功能, 1 - 使能BK9524红外码输出功能
#define DEFAULT_FREQ 7           // 启动默认频点

#if USE_BK9524_IR_FUNCTION
/*
这里是演示使用BK9524芯片输出红外键值。非BK9524用户不需要关心。
*/
#define IR_CustomCode 0x4321 // 红外用户码0x2143, 用户根据需要自行修改。

#endif

#ifndef __FREQTABLE_C__
extern USER_DATA_DEF USER_DATA;
extern unsigned int FreqTableA[];
extern unsigned int FreqTableB[];
#endif

#endif
