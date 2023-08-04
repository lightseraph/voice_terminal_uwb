#include "key.h"
#include "bk9535.h"
#include "tim.h"
#include "syscall.h"
#include "eeprom.h"
#include "ssd1306.h"
#include <stdio.h>

KEY_PROCESS_TypeDef key[KEYS];

void KEY_Process(int key_num)
{
    switch (key[key_num].flag.key_state)
    {
        // 【按键按下】
    case KEY_STATE_PRESS:
        // 在按键按下时从0开始计时，直到超时
        if (key[key_num].time_continus < KEY_TIME_OUT)
        {
            key[key_num].time_continus++;
        }

        // 发生长按事件
        if (key[key_num].time_continus > KEY_TIME_CONTINUS)
        {
            if (key[key_num].event_current_type != EVENT_NONE_CLICK)
            { // 识别长按前的按键事件
                if (key[key_num].press_cnt > 1)
                {
                    key[key_num].press_cnt--;
                }
                key[key_num].flag.once_event = 1;
            }
            else
            {
                key[key_num].flag.press_time = 1;             // 【0：短按/1：长按】识别此次为长按
                key[key_num].flag.key_state = KEY_STATE_IDLE; // 主动结束按下动作，进入无动作状态，保证在长按的按下过程中就识别出长按事件

                key[key_num].event_current_type = EVENT_LONG_CLICK; // 分配当前按键事件类型
                key[key_num].flag.once_event = 1;                   // 产生按键事件
                key[key_num].press_cnt = 1;
                key[key_num].time_idle = KEY_TIME_OUT; // 按键空闲时间超时
            }
        }

        // 按下时进行一次判断
        if (key[key_num].flag.check)
        {
            key[key_num].flag.check = 0;
            if (!key[key_num].flag.press_time)
            { // 判断上一次按键类型
                // 判断上一次按键动作空闲时间
                if (key[key_num].time_idle < KEY_TIME_IDLE)
                { // 若上一次按键动作后的空闲时间在规定时间内，说明发生了连击事件，一次完整的按键事件还未结束
                    key[key_num].press_cnt++;
                }
                else
                {
                    key[key_num].press_cnt = 1;
                }
            }
            key[key_num].flag.press_time = 0; // 【0：短按/1：长按】此次为短按
        }
        break;

        // 【按键松开】，若是长按，不会进入该判断
    case KEY_STATE_RELEASE:
        // 在按键按下时从0开始计时，直到超时
        if (key[key_num].time_idle < KEY_TIME_OUT)
        {
            key[key_num].time_idle++;
        }

        // 松开时进行一次判断
        if (key[key_num].flag.check)
        {
            key[key_num].flag.check = 0;

            // 判断此次按键动作
            if (!key[key_num].flag.press_time)
            { /// 长按会屏蔽短按
                if (key[key_num].press_cnt > 1)
                {                                                         // 连击事件
                    key[key_num].event_current_type = EVENT_DOUBLE_CLICK; // 分配按键事件类型
                    if (key[key_num].press_cnt > 4)
                        key[key_num].event_current_type = EVENT_MORE_CLICK;
                }
                else
                {                                                        // 单击事件
                    key[key_num].event_current_type = EVENT_SHORT_CLICK; // 分配按键事件类型
                    key[key_num].press_cnt = 1;                          // 连击次数置1
                }
            }
        }

        // 按键松开后判断此次按键动作后的空闲时间，从而判断此次动作是否结束
        if (key[key_num].time_idle > KEY_TIME_IDLE)
        { // 空闲时间超时，认为一次完整的按键事件结束
            if (!key[key_num].flag.press_time)
            {                                                 // 松开前是短按标志，则产生按键事件，这里是为了屏蔽长按后的松手动作
                key[key_num].flag.once_event = 1;             // 产生按键事件
                key[key_num].flag.key_state = KEY_STATE_IDLE; // 进入无动作状态
            }
        }
        break;
        // 【按键无动作】
    default:
        break;
    }
}

void KEY_Scan(void)
{
    for (int i = 0; i < KEYS; i++)
    {
        if (key[i].flag.once_event)
        {
            key[i].flag.once_event = 0;

            switch (key[i].event_current_type)
            {
            case EVENT_SHORT_CLICK:
                switch (i)
                {
                case 0:
                    SwitchNextFreq();
                    Flash_LED(LED_GREEN, 100, USER_DATA.rUserFreqIndex, FOLLOW_PREVIOUS);
                    AT24CXX_WriteOneByte(FREQ_ADDR, (u8)USER_DATA.rUserFreqIndex);
                    Disp_Freq(USER_DATA.rUserFreqIndex);
                    // HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
                    break;
                case 1:
                    SwitchPrevFreq();
                    Flash_LED(LED_GREEN, 100, USER_DATA.rUserFreqIndex, FOLLOW_PREVIOUS);
                    AT24CXX_WriteOneByte(FREQ_ADDR, (u8)USER_DATA.rUserFreqIndex);
                    Disp_Freq(USER_DATA.rUserFreqIndex);
                    // HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
                    break;
                }
                break;
            case EVENT_DOUBLE_CLICK:
                switch (i)
                {
                case 0:
                    local_id++;
                    if (local_id == 16)
                        local_id = 0;
                    Flash_LED(LED_GREEN, 100, 2, FOLLOW_PREVIOUS);
                    AT24CXX_WriteOneByte(LOCAL_ID_ADDR, local_id);
                    Disp_ID(local_id);
                    break;
                case 1:
                    if (local_id == 0)
                        local_id = 15;
                    else
                        local_id--;
                    Flash_LED(LED_GREEN, 100, 2, FOLLOW_PREVIOUS);
                    AT24CXX_WriteOneByte(LOCAL_ID_ADDR, local_id);
                    Disp_ID(local_id);
                    break;
                }
                break;
            case EVENT_MORE_CLICK:
                switch (i)
                {
                case 0:
                    Init_Param();
                    Flash_LED(LED_RED, 100, 3, FOLLOW_PREVIOUS);
                    break;
                case 1:
                    break;
                }
                break;
            case EVENT_LONG_CLICK:
                switch (i)
                {
                case 0:
                    HAL_GPIO_TogglePin(RF_CE_GPIO_Port, RF_CE_Pin);
                    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
                    delay_ms(1);
                    if (HAL_GPIO_ReadPin(RF_CE_GPIO_Port, RF_CE_Pin))
                    {
                        if (BK_Init())
                            Flash_LED(LED_GREEN, 50, 10, LIGHT_ON);

                        t_PCMCfg cfg;
                        cfg.bclk = PCM_SCK_I;
                        cfg.dat = PCM_SDA_I;
                        cfg.ch = RIGHT_CHANNEL;
                        cfg.mode = PCM_SLAVE;
                        cfg.lrck = PCM_LRCK_I;
                        BK_Tx_I2SOpen(cfg);
                        SwitchFreqByIndex(USER_DATA.rUserFreqIndex);
                        TX_WriteID(USER_DATA.UserId.dword);
                        TX_Trigger();
                    }
                    break;
                case 1:
                    break;
                }
                break;
            default:
                // printf("none\r\n");
                break;
            }
            // 事件处理完需更新前态和现态
            key[i].event_previous_type = key[i].event_current_type;
            key[i].event_current_type = EVENT_NONE_CLICK;
        }
    }
}

void KEY_Config(void)
{
    // 初始化
    for (int i = 0; i < KEYS; i++)
    {
        key[i].flag.check = 0;
        key[i].flag.key_state = KEY_STATE_IDLE;
        key[i].flag.once_event = 0;
        key[i].flag.press_time = 0;

        key[i].event_current_type = EVENT_NONE_CLICK;
        key[i].event_previous_type = EVENT_NONE_CLICK;
        key[i].press_cnt = 1;

        key[i].time_continus = 0;
        key[i].time_idle = KEY_TIME_OUT;
    }

    // 使能定时器
    HAL_TIM_Base_Start_IT(&htim6);
}
