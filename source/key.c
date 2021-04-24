/*
 * key.c
 *
 *  Created on: 2021年4月24日
 *      Author: shiyu
 */

#include "MK50D10.h"
#include "key.h"
#include "parameters.h"

const struct Key_Manage gKey_Manage[KEY_NUM] = {
	{{GPIOA, A_START_STOP_BUTTON},	KEY_SW2_VALUE}
};

/************************************************//**
* @brief 		IO初始化，因为在gpioSetup()初始化过了，所以这个函数不使用，
* 				如果使用这个函数，在函数中对gKey_Manage中的所有io初始化
* @param[in]
* @return
*************************************************/
void KEY_Init(void)
{

}

/************************************************//**
* @brief 		扫描所有的io，如果有按键按下，返回按键的Key_value
* @param[in]
* @return
*************************************************/
int KEY_Scan(void)
{
	int i = 0;
    int key = 0;

	for(i=0; i<KEY_NUM; i++)
	{
		if( (gKey_Manage[i].pin.gpio_type->PDIR & gKey_Manage[i].pin.gpio_num) == 0 )
		{
			key |= gKey_Manage[i].Key_value;
		}
	}

    return key;
}

/************************************************//**
* @brief 		按键状态机，这个函数10ms调用一次
* @param[in]
* @return
*************************************************/
int Key_Driver(void)
{
    static e_Key_State key_state = NULL_STATE;	// 当前状态，注意要用static
    static uint8_t key_time = 0;				// 按键按下计时，注意要用static
	static int key_value = 0;					// 当前的按键值，注意要用static
	int key_return = 0;							// 最终的按键值

	switch(key_state)
	{
		case NULL_STATE:	//空闲状态
		{
			// 检测按键是否按下
			key_value = KEY_Scan();
			if( key_value != 0x00 )
			{
				key_state = ACK_STATE;	//跳到确认状态
				key_time = 0;
			}
		}
		break;
		case ACK_STATE:		//确认状态
		{
			// 如果还是按下的，计时
			if( KEY_Scan() != 0x00 )
			{
				key_time ++;
				if( key_time > 200 )	// 如果连续按下2S，说明是长按
				{
					key_state = LONG_PRESS_STATE;	//跳到长按状态
				}
			}
			else	// 如果按键松开了，可能是短按，也可能是双击
			{
				key_state = JUDGMENT_STATE;		//跳到单双击判定状态
				key_time = 0;
			}
		}
		break;
		case JUDGMENT_STATE:	//单双击判定状态
		{
			// 如果按键又按下了，说明是双击
			if( KEY_Scan() != 0x00 )
			{
				key_state = DOUBLE_CLICK_STATE;	//跳到双击状态
			}
			else
			{
				key_time ++;
				if( key_time > 20 )	// 如果200ms内没有再次按下，说明是短按
				{
					key_state = SHORT_PRESS_STATE;	//跳到短按状态
				}
			}
		}
		break;
		case SHORT_PRESS_STATE:	//短按状态
		{
			key_return = key_value;
			key_state = WAIT_RELEASE_STATE;
		}
		break;
        case LONG_PRESS_STATE:	//长按状态
        {
        	key_return = key_value | LONG_PRESS;
        	key_state = WAIT_RELEASE_STATE;
        }
        break;
        case DOUBLE_CLICK_STATE:	//双击状态
        {
        	key_return = key_value | DOUBLE_CLICK;
        	key_state = WAIT_RELEASE_STATE;
        }
        break;
        case WAIT_RELEASE_STATE:	// 等待按键释放状态
        {
        	if( KEY_Scan() == 0x00 )
        	{
        		key_state = NULL_STATE;	// 如果按键释放，跳到空闲状态，一次按键检测就完成了
        	}
        }
        break;
        default : break;
	}
	return key_return;	// 返回按键的值
}

