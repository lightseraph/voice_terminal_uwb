#include "eeprom.h"
#include "syscall.h"

void Init_Param(void)
{
    uint8_t temp = 0;
    AT24CXX_WriteOneByte(LOCAL_ID_ADDR, temp); // 默认出厂设置id索引号
    temp = 1;
    AT24CXX_WriteOneByte(FREQ_ADDR, temp); // 默认出厂设置频点索引号
    delay_ms(50);
}
