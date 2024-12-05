#include <Arduino.h>
#include <driver/uart.h>
#include <soc/uart_reg.h>
#include <soc/uart_struct.h>
#include <esp_log.h>

#include "CRC16.h"
#include "FRAME_PARSE.h"
#include "dev_act_hdl.h"

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#define THINGSBOARD_ENABLE_PROGMEM 0
#elif defined(ESP32) || defined(RASPBERRYPI_PICO) || defined(RASPBERRYPI_PICO_W)
#include <WiFi.h>
#endif

#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>