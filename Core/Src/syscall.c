#include "syscall.h"
#include "usart.h"
#include "tim.h"
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

vs8 keypress_remain = 0;

void Flash_LED(LED_TYPE led, u16 interval, u8 count, LED_AFTER_FLASH cond)
{
    for (int i = 0; i < count * 2; i++)
    {
        if (led == LED_RED)
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        else if (led == LED_GREEN)
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

        delay_ms(interval);
    }
    if (cond != FOLLOW_PREVIOUS)
    {
        if (led == LED_RED)
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (GPIO_PinState)cond);
        else if (led == LED_GREEN)
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (GPIO_PinState)cond);
    }
}

int _write(int file, char *data, int len)
{
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
    {
        errno = EBADF;
        return -1;
    }

    // arbitrary timeout 1000
    HAL_StatusTypeDef status =
        HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 1000);

    // return # of bytes written - as best we can tell
    return (status == HAL_OK ? len : 0);
}

void delay_us(u16 num)
{
    __HAL_TIM_SET_COUNTER(&htim7, 0); // 将装载值计0
    HAL_TIM_Base_Start(&htim7);       // 开始计数
    while (__HAL_TIM_GetCounter(&htim7) < num)
        ;                      // 当装载值小于预设值时循环
    HAL_TIM_Base_Stop(&htim7); // 结束延时
}

void delay_ms(u16 num)
{
    for (int i = 0; i < num; i++)
    {
        delay_us(1000);
    }
}