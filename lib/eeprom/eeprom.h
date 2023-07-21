#ifndef __EEPROM_H
#define __EEPROM_H

#include "main.h"
#include <string.h>
#define iEEPROM_CHECK_NUM 2

#define EEPROM_BASE_ADDR (0x08080000U)
#define EEPROM_BYTE_SIZE (0x07FF)

#define LOCAL_ID_ADDR 0x00
#define FREQ_ADDR 0x01

void EEPROM_WRITE(uint16_t BiasAddress, uint8_t *Data, uint16_t len);
void EEPROM_READ(uint16_t BiasAddress, uint8_t *Buffer, uint16_t len);
HAL_StatusTypeDef EEPROM_WRITE_W_CHECK(uint16_t BiasAddress, uint8_t *Data, uint16_t len);
HAL_StatusTypeDef EEPROM_Read_W_CHECK(uint16_t BiasAddress, uint8_t *Data, uint16_t len);

#endif