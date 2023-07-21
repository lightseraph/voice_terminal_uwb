#include "syscall.h"
#include "usart.h"
#include "tim.h"
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

vs8 keypress_remain = 0;

void delay_nus(u32 nus)
{
    if (SysTick_Config(80))
    {
        while (1)
            ;
    }
    time_delay = nus; // 读取定时时间
    while (time_delay)
        ;
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // 关闭计数器
    SysTick->VAL = 0X00;                       // 清空计数器
}

void delay_nms(u32 nms)
{
    if (SysTick_Config(SystemCoreClock / 1000UL - 1))
    {
        while (1)
            ;
    }
    time_delay = nms; // 读取定时时间
    while (time_delay)
        ;
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // 关闭计数器
    SysTick->VAL = 0X00;                       // 清空计数器
}

void Flash_LED(LED_TYPE led, u16 interval, u8 count, LED_AFTER_FLASH cond)
{
    /* for (int i = 0; i < count * 2; i++)
    {
        if (led == LED_RED)
            HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
        else if (led == LED_GREEN)
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

        delay_nms(interval);
    }
    if (cond != FOLLOW_PREVIOUS)
    {
        if (led == LED_RED)
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, (GPIO_PinState)cond);
        else if (led == LED_GREEN)
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, (GPIO_PinState)cond);
    } */
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

void IR_delay(uint16_t num)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0); // 将装载值计0
    HAL_TIM_Base_Start(&htim6);       // 开始计数
    while (__HAL_TIM_GetCounter(&htim6) < num)
        ;                      // 当装载值小于预设值时循环
    HAL_TIM_Base_Stop(&htim6); // 结束延时
}