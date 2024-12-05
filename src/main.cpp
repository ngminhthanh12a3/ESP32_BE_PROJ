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

// Your WiFi credentials.
// Set password to "" for open networks.
constexpr char WIFI_SSID[] = "DESIoT";
constexpr char WIFI_PASSWORD[] = "deslab2610";

constexpr char TOKEN[] = "mMHaXYQblAvbljdl3w38";

// Thingsboard we want to establish a connection too
constexpr char THINGSBOARD_SERVER[] = "tb.desiot.io.vn";
// MQTT port used to communicate with the server, 1883 is the default unencrypted MQTT port.
constexpr uint16_t THINGSBOARD_PORT = 20004U;

// Maximum size packets will ever be sent or received by the underlying MQTT client,
// if the size is to small messages might not be sent or received messages will be discarded
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

// Baud rate for the debugging serial connection.
// If the Serial output is mangled, ensure to change the monitor speed accordingly to this variable
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

// Maximum amount of attributs we can request or subscribe, has to be set both in the ThingsBoard template list and Attribute_Request_Callback template list
// and should be the same as the amount of variables in the passed array. If it is less not all variables will be requested or subscribed
constexpr size_t MAX_ATTRIBUTES = 3U;

constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 5000U * 1000U;

// Initialize underlying client, used to establish a connection
WiFiClient wifiClient;

Arduino_MQTT_Client mqttClient(wifiClient);

// Initialize ThingsBoard instance with the maximum needed buffer size, stack size and the apis we want to use
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE, Default_Max_Stack_Size);

// Settings for interval in blinking mode
constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

uint32_t previousStateChange;

// For telemetry
constexpr int16_t telemetrySendInterval = 2000U;
uint32_t previousDataSend;

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    // Delay 500ms until a connection has been succesfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

/// @brief Reconnects the WiFi uses InitWiFi if the connection has been removed
/// @return Returns true as soon as a connection has been established again
const bool reconnect()
{
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED)
  {
    return true;
  }

  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

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
}

void loop()
{
  // put your main code here, to run repeatedly:
  delay(10);
  if (!reconnect())
  {
    return;
  }

  if (FrameParse.FP_MOD == DECT_COML) // Always check in loop
  {
    HandlerDeviceAction(FrameParse._CMD,
                        FrameParse.buffer.len,
                        FrameParse.buffer.data, &tb);
    FP_Init(&FrameParse);
  }

  if (!tb.connected())
  {
    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT))
    {
      Serial.println("Failed to connect");
      return;
    }
    // Sending a MAC address as an attribute
    tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
  }
  if (millis() - previousDataSend > telemetrySendInterval)
  {
    previousDataSend = millis();
    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
  }

  tb.loop();
}