/*
 * key.h
 *
 *  Created on: 2021年4月24日
 *      Author: shiyu
 */

#ifndef SOURCE_KEY_H_
#define SOURCE_KEY_H_

#include "MK50D10.h"

#define KEY_NUM (1)
#define LONG_PRESS (0x100)
#define DOUBLE_CLICK (0x200)

#define KEY_SW2_VALUE	(0x01)

// GPIO对像
struct Gpio_Fun
{
	GPIO_Type *gpio_type;
	uint32_t gpio_num;
};

// key对像
struct Key_Manage
{
	const struct Gpio_Fun pin;
	const int Key_value;
};

//按键状态
typedef enum
{
    NULL_STATE,			// 空闲状态
    ACK_STATE,			// 确认状态
	JUDGMENT_STATE,		// 单双击判定状态
	SHORT_PRESS_STATE,	// 短按状态
    LONG_PRESS_STATE,	// 长按状态
	DOUBLE_CLICK_STATE,	// 双击状态
	WAIT_RELEASE_STATE,	// 等待按键释放
}e_Key_State;

void KEY_Init(void);
int KEY_Scan(void);
int Key_Driver(void);


#endif /* SOURCE_KEY_H_ */
