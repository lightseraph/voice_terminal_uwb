#ifndef __EEPROM_H
#define __EEPROM_H
#include "24cxx.h"

#define EEPROM_DATAFORMAT (0xB1)

#define RF_PARAM_BASEADDR (0x0FE0)
#define LOCAL_ID_ADDR (RF_PARAM_BASEADDR + 0x0)
#define FREQ_ADDR (RF_PARAM_BASEADDR + 0x1)

#define UWB_PARAM_BASEADDR (0x0001)
#define UWB_PARAM_DEVICEADDR (UWB_PARAM_BASEADDR + 0x0)
#define UWB_PARAM_TA_KAL (UWB_PARAM_BASEADDR + 0x2)

void Init_Param(void);
uint8_t rom_DeviceMode(void);
uint16_t rom_DeviceAddr(void);
uint8_t rom_KalFilter(void);

#endif