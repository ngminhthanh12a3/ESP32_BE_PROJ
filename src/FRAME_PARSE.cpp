#include <Arduino.h>
#include "FRAME_PARSE.h"

extern FrameParse_t FrameParse;

uint8_t FP_HEADER[] = {FP_H1, FP_H2, FP_T1, FP_T2};
uint8_t DectModHandler(FrameParse_t *FP_handler, uint8_t __char)
{
	if (__char == FP_HEADER[FP_handler->H_Index])
	{
		// go to next header
		if ((++FP_handler->H_Index) == sizeof(FP_HEADER))
			FP_handler->H_Index = 0;

		return FP_OK;
	}
	return FP_FAIL;
}

void FP_Init(FrameParse_t *FP_handler)
{
	// initialize fields
	FP_handler->FP_MOD = DECT_H_MOD;
	FP_handler->FP_Status = FP_OK;

	//
	FP_handler->H_Index = 0;
	FP_handler->_CRC.index = 0;
	FP_handler->buffer.index = 0;
}
/**
 * @brief  Handle Frame Parse Status Fail.
 * @note   .
 * @param  __char character.
 * @retval
 */
void HandleFPStatusFail(FrameParse_t *FP_handler)
{
	FP_Init(FP_handler);
}

uint8_t FP_CheckCRC(FrameParse_t *FP_handler)
{
	uint16_t FRAME_CRC = (FP_handler->_CRC.buffer[1] << 8) | FP_handler->_CRC.buffer[0];

	uint16_t CHECK_CRC = Compute_CRC16(FP_handler->buffer.data, FP_handler->buffer.len);
	return CHECK_CRC == FRAME_CRC;
}

void ParseFrameHandler(FrameParse_t *FP_handler, uint8_t __char)
{
	// pre handlers

	// Status fail
	if (FP_handler->FP_Status == FP_FAIL)
	{
		// handlers
		HandleFPStatusFail(FP_handler);
	}

	//
	FP_handler->cur_char = __char;
	switch (FP_handler->FP_MOD)
	{
	case DECT_H_MOD:
		FP_handler->FP_Status = DectModHandler(FP_handler, __char);

		// go to next mode
		if (FP_HEADER[FP_handler->H_Index] == FP_T1)
			FP_handler->FP_MOD = DECT_CMD;
		if (FP_HEADER[FP_handler->H_Index] == FP_H1)
		{
			// case that the buffer has buffer length
			if (FP_handler->buffer.len)
				FP_handler->FP_MOD = COL_CRC;
			else
				FP_handler->FP_MOD = DECT_COML;
		}

		break;
	case DECT_CMD:
		//			FP_handler->dev.id = __char & 0x7Fu;
		//			FP_handler->dev.mode = __char >> 7;
		FP_handler->_CMD = __char;

		// go to next mode
		FP_handler->FP_MOD = DECT_BUF_LEN;
		break;
	case DECT_BUF_LEN:
		FP_handler->buffer.len = __char;

		//
		FP_handler->buffer.index = 0;

		// go to next mode
		if (FP_handler->buffer.len)
			FP_handler->FP_MOD = COL_BUF_DATA;
		else
			FP_handler->FP_MOD = DECT_H_MOD;
		break;
	case COL_BUF_DATA:
		if (FP_handler->buffer.len)
			FP_handler->buffer.data[FP_handler->buffer.index] = __char;

		// increase buffer index, and go to next mode
		if ((++FP_handler->buffer.index) == FP_handler->buffer.len)
		{
			FP_handler->FP_MOD = DECT_H_MOD;
		}
		break;
	case COL_CRC:
		FP_handler->_CRC.buffer[FP_handler->_CRC.index] = __char;

		// increase buffer index, and go to next mode
		if ((++FP_handler->_CRC.index) == sizeof(FP_handler->_CRC.buffer))
		{
			if (FP_CheckCRC(FP_handler))
			{
				FP_handler->FP_MOD = DECT_COML;
				// Serial.printf("\nFP_CheckCRC complete");
			}
			else
			{
				HandleFPStatusFail(FP_handler);
			}
		}
		break;
	default:
		break;
	}
}
