#ifndef __BK9531_H__
#define __BK9531_H__

#include "config.h"
#include "ctiic.h"
#include "main.h"

#define STATE_TsdIdle 0
#define STATE_TsdStart 1
#define STATE_TsdDataLo 2
#define STATE_TsdDataHi 3
#define STATE_TsdChkLo 4
#define STATE_TsdChkHi 5

#define LEAD_TsdIdle 0xff
#define LEAD_TsdStart 0x00
#define LEAD_TsdDataLo 0x6 // 0110b
#define LEAD_TsdDataHi 0x9 // 1001b
#define LEAD_TsdChkLo 0x5  // 0101b
#define LEAD_TsdChkHi 0xa  // 1010b

#define CHIP_DEV_TX 0x25
#define TX_Chip_ID 0x35
#define TX_INIT_RETRY_TIMES 10
#define TX_I2C_RETRY_TIMES 3

#define TRIGGER_RETRY 3

#ifdef __BK9531_ProC__
#define __TX_EXT__
// static unsigned char LinkIDTmp[4];
unsigned char rTsdState;
#else
#define __TX_EXT__ extern
#endif

#define TX_POWER_VBAND_14DBM 14
#define TX_POWER_VBAND_12DBM 12
#define TX_POWER_VBAND_10DBM 10
#define TX_POWER_VBAND_8DBM 8
#define TX_POWER_VBAND_6DBM 6
#define TX_POWER_VBAND_4DBM 4
#define TX_POWER_VBAND_2DBM 2
#define TX_POWER_VBAND_0DBM 0

#define TX_POWER_UBAND_10DBM 10
#define TX_POWER_UBAND_8DBM 8
#define TX_POWER_UBAND_6DBM 6
#define TX_POWER_UBAND_4DBM 4
#define TX_POWER_UBAND_2DBM 2
#define TX_POWER_UBAND_0DBM 0

__TX_EXT__ u8 reg_val[4];                                // Temporatory register values
__TX_EXT__ volatile unsigned char analog_reg_val[12][4]; // Analog register values,
__TX_EXT__ unsigned char rWorkChannel;
__TX_EXT__ unsigned char rTriggerValue;

__TX_EXT__ u8 BK_Init(void);
__TX_EXT__ void TX_Reset_Chip(void);
__TX_EXT__ void TX_SingleWave_Start(void);
__TX_EXT__ void TX_SingleWave_Stop(void);
__TX_EXT__ void TX_PN9_Start(void);
__TX_EXT__ void TX_PN9_Stop(void);
__TX_EXT__ void Reset_chip_tx(void);
__TX_EXT__ void TX_TuneFreq(unsigned long khz);
__TX_EXT__ void SwitchFreqByIndex(unsigned char index);
__TX_EXT__ void SwitchNextFreq(void);
__TX_EXT__ void SwitchPrevFreq(void);
__TX_EXT__ void TX_I2C_WriteReg(unsigned char reg, unsigned long dat);
__TX_EXT__ unsigned long TX_I2C_ReadReg(unsigned char reg);
__TX_EXT__ void TX_WriteUserData(unsigned char dat);
__TX_EXT__ void TX_Trigger();
__TX_EXT__ void TX_Prevent_RF_UnLock(void);
__TX_EXT__ void Set_TX_Volume(unsigned char vol);
__TX_EXT__ void EnableAudioIndicator(void);
__TX_EXT__ u8 TX_I2C_Write(u8 reg, u8 *buf);
__TX_EXT__ u8 TX_I2C_Read(u8 reg, u8 *buf);
__TX_EXT__ void Adjust_XTAL(u8 value);

#define TX_WriteID(id) (TX_I2C_WriteReg(0x38, id))

#if USE_I2S_AUDOI
__TX_EXT__ void BK_Tx_I2SOpen(t_PCMCfg cfg);
#endif

__TX_EXT__ void SamplingRateSel(e_WorkMode mode);

#if USE_BK9524_IR_FUNCTION
__TX_EXT__ void SendIrCode(void);
__TX_EXT__ void IrCodeStartSend(unsigned char keycode);
#endif

#endif
