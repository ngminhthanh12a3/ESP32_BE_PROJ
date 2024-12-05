#include "dev_act_hdl.h"
volatile int red, ir, hr, spo2;
// MAX30102
const int32_t ledsBufferLength = 100; // data length
uint8_t IR_OFFSET = 100;
#define M_FUL_BUF_SIZE (200u + 4u)
#define M_32_FUL_BUF_SIZE (M_FUL_BUF_SIZE / 2u)

#if !(defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__))
// Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
// To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
// uint16_t irBuffer[ledsBufferLength + 1]; // infrared LED sensor data
// uint16_t redBuffer[ledsBufferLength + 1]; // red LED sensor data
uint16_t MAX30102_FUL_BUF[M_FUL_BUF_SIZE];
#else
uint32_t MAX30102_FUL_BUF[M_FUL_BUF_SIZE];
// uint32_t irBuffer[ledsBufferLength + 1]; // infrared LED sensor data
// uint32_t redBuffer[ledsBufferLength + 1]; // red LED sensor data
#endif
uint16_t ledBufIndex = 0;
// int32_t MAX30102_O_BUF[4];
// int32_t spo2 = 0;		   // SPO2 value
// int8_t validSPO2 = 0;	   // indicator to show if the SPO2 calculation is valid
// int32_t heartRate = 0;	   // heart rate value
// int8_t validHeartRate = 0; // indicator to show if the heart rate calculation is valid

void Flip(uint8_t buffer_len, uint8_t *buffer)
{
	for (uint8_t i = 0; i < buffer_len / 2; i++)
	{
		uint8_t temp = buffer[i];
		buffer[i] = buffer[buffer_len - i - 1];
		buffer[buffer_len - i - 1] = temp;
	}
}

void HandlerDeviceAction(uint8_t CMD, uint8_t buffer_len, uint8_t *buffer, ThingsBoard *cur_tb)
{
	// Serial.printf("\nFP_ Handle");
	uint8_t devID = CMD & 0x7Fu;
	switch (devID)
	{
	case DEV_0_ID:
	{

		ledBufIndex = *(buffer + buffer_len - 1);

		red = *((uint16_t *)buffer + 0);
		ir = *((uint16_t *)buffer + 1);
		// cur_Bylnk->virtualWrite(V2, red);
		// cur_Bylnk->virtualWrite(V3, ir);
		cur_tb->sendTelemetryData("red", red);
		cur_tb->sendTelemetryData("ir", ir);

		if (buffer_len == 13)
		{
			spo2 = *((uint32_t *)buffer + 1);
			hr = *((uint32_t *)buffer + 2);
			// cur_Bylnk->virtualWrite(V0, hr);
			// cur_Bylnk->virtualWrite(V1, spo2);
			cur_tb->sendTelemetryData("heartRate", hr);
			cur_tb->sendTelemetryData("spo2", spo2);
		}
		break;
	}
	case DEV_1_ID:
		break;
	default:
		break;
	}
}
