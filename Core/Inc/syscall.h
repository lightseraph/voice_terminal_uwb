#ifndef __SYSCALL_H
#define __SYSCALL_H

#include "main.h"

extern vs8 keypress_remian;
typedef enum
{
    LED_RED,
    LED_GREEN
} LED_TYPE;

typedef enum
{
    LIGHT_ON = 0,
    LIGHT_OFF,
    FOLLOW_PREVIOUS,
} LED_AFTER_FLASH;

void Flash_LED(LED_TYPE led, u16 interval, u8 count, LED_AFTER_FLASH cond);

void delay_nus(u32 num);
void delay_nms(u32 num);
void IR_delay(uint16_t num);

#endif