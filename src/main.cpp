#include "main.h"

// UART PV
static intr_handle_t handle_console;
const uart_port_t uart_num = UART_NUM_2;
const int uart_buffer_size = (1024);
static uint8_t ucParameterToPass;
TaskHandle_t xHandle = NULL;

// Frame parse PV
FrameParse_t FrameParse;

uint32_t value = 0;
uint8_t startToCalculate = 0;
uint8_t needToUpdate = 0;

extern uint8_t ledBufIndex;

char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "DESIoT";
char pass[] = "deslab2610";
BlynkTimer timer;
static void UART_ISR_ROUTINE(void *pvParameters)
{
  // uart_event_t event;
  // size_t buffered_size;
  // uint8_t dtmp[1];
  FrameParse_t *FrameParse = (FrameParse_t *)pvParameters;

  for (;;)
  {
    // Task code goes here.
    const uart_port_t uart_num = UART_NUM_2;
    uint8_t data[128];
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *)&length));
    if (length)
    {
      length = uart_read_bytes(uart_num, data, length, 1000);
      // Serial.printf("\nLength: %d\n", length);
      // Serial.write(data, length);
      for (uint8_t i = 0; i <= length;)
      {
        if (FrameParse->FP_MOD != DECT_COML)
        {
          ParseFrameHandler(FrameParse, data[i]);
          i++;
        }
        // else
        // {
        //   Serial.printf("\r\nWaiting for Frame complete");
        //   delay(1000);
        // }
      }
      // Serial.printf("\r\nEnd read UART");
    }
    // if (length)
    // delay(1);
  }
}

void myTimerEvent() // This loop defines what happens when timer is triggered
{
  // Serial.printf("\r\nReal connected: %d", Blynk.connected());
}

void setup()
{
  // put your setup code here, to run once:

  Serial.begin(115200, SERIAL_8N1, 3, 1, 0, 20000UL, 112U);

  // FRAME PART setup
  FP_Init(&FrameParse);

  // // UART setup
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
  };
  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

  //
  // Set UART pins (using UART0 default pins ie no changes.)
  ESP_ERROR_CHECK(uart_set_pin(uart_num, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  //
  QueueHandle_t uart_queue;
  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size,
                                      0, 10, &uart_queue, 0));
  // clear the buffer
  // uart_flush(uart_num);

  //
  // ESP_ERROR_CHECK(uart_isr_free(uart_num));                                                                  // release the pre registered UART handler/subroutine
  // ESP_ERROR_CHECK(uart_isr_register(uart_num, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console)); // register new UART subroutine
  // ESP_ERROR_CHECK(uart_enable_rx_intr(uart_num));
  xTaskCreate(UART_ISR_ROUTINE, "UART_ISR_ROUTINE", 8192, &FrameParse, 1, NULL); // enable RX interrupt

  Blynk.begin(auth, ssid, pass);
  timer.setInterval(1000L, myTimerEvent); // Staring a timer
}

void loop()
{
  // put your main code here, to run repeatedly:
  Blynk.run();
  timer.run();

  if (FrameParse.FP_MOD == DECT_COML) // Always check in loop
  {
    HandlerDeviceAction(FrameParse._CMD,
                        FrameParse.buffer.len,
                        FrameParse.buffer.data, &Blynk);
    FP_Init(&FrameParse);
  }
}