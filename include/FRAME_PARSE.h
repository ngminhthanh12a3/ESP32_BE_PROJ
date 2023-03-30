#ifndef __FRAME_PARSE_H__
#define __FRAME_PARSE_H__

#include <stdint.h>
#include "CRC16.h"

#define FP_MAX_BUF_SIZE 255u

#define FP_H1 0xABu
#define FP_H2 0xCDu
#define FP_T1 0xE1u
#define FP_T2 0xE2u

// Length in byte unit
#define FP_H1_LEN 			1u
#define FP_H2_LEN 			1u
#define FP_H_LEN 			FP_H1_LEN + FP_H2_LEN
#define FP_T1_LEN 			1u
#define FP_T2_LEN 			1u
#define FP_T_LEN 			FP_T1_LEN + FP_T2_LEN
#define FP_CMD_LEN 			1u
#define FP_DL_LEN 			1u // Data Length
#define FP_MAX_DL_LEN 		255u // Max Data Length
#define FP_CRC_LEN 			2u
#define FP_F_LEN(DATA_LEN) 	(FP_H_LEN + \
							FP_CMD_LEN + \
							FP_DL_LEN + DATA_LEN + \
							FP_T_LEN + \
							FP_CRC_LEN) //Frame length

#define FP_F_MAX_LEN 	FP_F_LEN(FP_MAX_DL_LEN)//Frame max length

enum FP_STATUS {
	FP_FAIL,
	FP_OK
};
enum FP_MODES {
	// FRAME HEADER DETECT MODE
	DECT_H_MOD,
	// COMMAND DETECT MODE
	DECT_CMD,
	// FRAME BUFFER LENGTH DETECT MODE
	DECT_BUF_LEN,
	// Collect buffer data
	COL_BUF_DATA,
	// Collect CRC
	COL_CRC,
	// DETECT COMPLETE MODE
	DECT_COML
};

typedef struct {
	uint8_t len;
	uint8_t data[FP_MAX_BUF_SIZE];
	uint8_t index;
}buffer_t;

//typedef struct {
//	uint8_t id;
//	uint8_t mode;
//}FP_dev_t;

typedef struct {
	uint8_t buffer[2];
	uint8_t index;
}FP_CRC_t;

typedef struct {
	// Frame Parse Mode
	uint8_t FP_MOD;
	// Frame Parse Status
	uint8_t FP_Status;

	// header index
	uint8_t H_Index;
	//
	uint8_t cur_char;

	//
	buffer_t buffer;

	//
//	FP_dev_t dev;
	uint8_t _CMD;

	//
	FP_CRC_t _CRC;
}FrameParse_t;

uint8_t DectModHandler(FrameParse_t *FP_handler, uint8_t __char);
void FP_Init(FrameParse_t *FP_handler);
void HandleFPStatusFail(FrameParse_t *FP_handler);
uint8_t FP_CheckCRC(FrameParse_t *FP_handler);
void ParseFrameHandler(FrameParse_t *FP_handler, uint8_t __char);

#endif //__FRAME_PARSE_H__
