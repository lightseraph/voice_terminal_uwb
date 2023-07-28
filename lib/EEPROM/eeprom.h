#ifndef __EEPROM_H
#define __EEPROM_H
#include "24cxx.h"

#define EEPROM_DATAFORMAT (0xB1)

#define RF_PARAM_BASEADDR (0xFE0)
#define LOCAL_ID_ADDR (RF_PARAM_BASEADDR + 0)
#define FREQ_ADDR (RF_PARAM_BASEADDR + 1)

#define UWB_PARAM_BASEADDR (0x001)

void Init_Param(void);

#endif