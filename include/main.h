#include <Arduino.h>
#include <driver/uart.h>
#include <soc/uart_reg.h>
#include <soc/uart_struct.h>
#include <esp_log.h>

//
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "CRC16.h"
#include "FRAME_PARSE.h"
#include "dev_act_hdl.h"

#define SERVICE_UUID "6f576859-4372-45ff-b05b-7527a51f6867"