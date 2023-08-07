#include "eeprom.h"
#include "syscall.h"

void Init_Param(void)
{
    uint8_t temp = 0;
    AT24CXX_WriteOneByte(LOCAL_ID_ADDR, temp); // 默认出厂设置id索引号
    temp = 2;
    AT24CXX_WriteOneByte(UWB_PARAM_TA_KAL, temp); // 1:基站，0：标签;    0010
    temp = 1;
    AT24CXX_WriteOneByte(FREQ_ADDR, temp); // 默认出厂设置频点索引号
    uint8_t addr[2] = {0x00, 0x02};        // 标签地址
    AT24CXX_WriteLenByte(UWB_PARAM_DEVICEADDR, addr, 2);

    delay_ms(50);
}

uint8_t rom_DeviceMode(void)
{
    uint8_t temp = AT24CXX_ReadOneByte(UWB_PARAM_TA_KAL);
    return (temp & 0x01);
}

uint16_t rom_DeviceAddr(void)
{
    uint16_t temp = (uint16_t)AT24CXX_ReadLenByte(UWB_PARAM_DEVICEADDR, 2);
    return temp;
}

uint8_t rom_KalFilter(void)
{
    uint8_t temp = AT24CXX_ReadOneByte(UWB_PARAM_TA_KAL);
    return (temp & 0x02);
}
