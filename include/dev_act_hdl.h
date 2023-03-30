#ifndef __DEV_ACT_HDL_H__
#define __DEV_ACT_HDL_H__

#include <stdint.h>
#include "main.h"
#include "CRC16.h"
#include "FRAME_PARSE.h"

//#define DEV_1_ID 0x1u

//
#define DEV_1_PORT GPIOB

//
#define DEV_1_PIN GPIO_PIN_13

enum DEV_IDs
{
	DEV_0_ID,
	DEV_1_ID,
	DEV_2_ID
};

enum DEV_DT_LEN
{
	DEV_0_DT_LEN = 0x4u
};

enum DEV_MOD
{
	//	Write Mode
	DEV_W_MOD,
	//	Read Mode
	DEV_R_MOD
};

void HandlerDeviceAction(uint8_t CMD, uint8_t buffer_len, uint8_t *buffer);

#endif //__DEV_ACT_HDL_H__
