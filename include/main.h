#define BLYNK_TEMPLATE_NAME "MAX30102"
#define BLYNK_TEMPLATE_ID "TMPL6kkwsshYu"
#define BLYNK_AUTH_TOKEN "Z45yX5VsFKCMe5P8tQBvJPqDWnIi7tgo"

#define BLYNK_TEMPLATE_ID "TMPL6kkwsshYu"
#define BLYNK_TEMPLATE_NAME "MAX30102"
#define BLYNK_AUTH_TOKEN "Z45yX5VsFKCMe5P8tQBvJPqDWnIi7tgo"
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <driver/uart.h>
#include <soc/uart_reg.h>
#include <soc/uart_struct.h>
#include <esp_log.h>

#include "CRC16.h"
#include "FRAME_PARSE.h"
#include "dev_act_hdl.h"

#include <BlynkSimpleEsp32.h>