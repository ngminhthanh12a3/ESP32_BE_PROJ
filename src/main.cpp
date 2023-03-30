#include "main.h"

// UART PV
static intr_handle_t handle_console;
const uart_port_t uart_num = UART_NUM_2;
const int uart_buffer_size = (1024);
static uint8_t ucParameterToPass;
TaskHandle_t xHandle = NULL;

// Frame parse PV
FrameParse_t FrameParse;

// BLE PV
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic[4] = {NULL};
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
uint8_t startToCalculate = 0;
uint8_t needToUpdate = 0;

// char SERVICE_UUID[] = "6f576859-4372-45ff-b05b-7527a51f6867";
const char CHARACTERISTIC_UUID[4][37] = {"b8eaccdb-8188-42a3-a10b-3b49efeb61fd", "c3d2fa8a-4358-4569-a346-484df10520f5",
                                         "d711ede2-ff90-4367-baca-be1ac6ddde52", "b2d483b9-64c0-4fb8-bf97-04aa66124bc5"};
extern uint8_t ledBufIndex;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    startToCalculate = *pCharacteristic->getData();
    Serial.printf("\nstart: %d", startToCalculate);
    needToUpdate = 1;
  }
};
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
        else
        {
          HandlerDeviceAction(FrameParse->_CMD,
                              FrameParse->buffer.len,
                              FrameParse->buffer.data);
          FP_Init(FrameParse);
          i++; // bug
        }
        // delay(1);
      }
    }
    // if (length)
    delay(1);
  }
}
static void BLE_TASK_HANDLER(void *pvParameters)
{
  for (;;)
  {
    // // notify changed value
    // if (deviceConnected)
    // {
    //   // pCharacteristic->setValue((uint8_t *)&value, 4);
    //   pCharacteristic->notify();
    //   // value++;
    //   delay(1000); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    // }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
      delay(500);                  // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      // Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
    }
    delay(1);
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200, SERIAL_8N1, 3, 1, 0, 20000UL, 112U);

  // FRAME PART setup
  FP_Init(&FrameParse);

  // UART setup
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

  // //
  // ESP_ERROR_CHECK(uart_isr_free(uart_num));                                                                  // release the pre registered UART handler/subroutine
  // ESP_ERROR_CHECK(uart_isr_register(uart_num, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console)); // register new UART subroutine
  // ESP_ERROR_CHECK(uart_enable_rx_intr(uart_num));
  xTaskCreate(UART_ISR_ROUTINE, "UART_ISR_ROUTINE", 8192, &FrameParse, 1, NULL); // enable RX interrupt

  // BLE settup
  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  for (uint8_t i = 0; i < 4; i++)
  {
    // Create a BLE Characteristic
    pCharacteristic[i] = pService->createCharacteristic(
        CHARACTERISTIC_UUID[i],
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE);

    // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
    // Create a BLE Descriptor
    pCharacteristic[i]->addDescriptor(new BLE2902());
  }
  // start to calculating setup
  pCharacteristic[DEV_1_ID]->setValue(&startToCalculate, 1);
  pCharacteristic[DEV_1_ID]->setCallbacks(new MyCharacteristicCallbacks());

  // Start the service
  pService->start();
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

  Serial.println("Waiting a client connection to notify...");
  // void *BLE_pParameters = {pServer, pC};
  xTaskCreate(BLE_TASK_HANDLER, "BLE_TASK_HANDLER", 8192, NULL, 1, NULL);
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (needToUpdate)
  {
    Serial.printf("\nStart to calculate");
    uint8_t data[] = {startToCalculate};
    // data[0] = (!data[0]) & 0x1;

    unsigned short CRC = Compute_CRC16(data, sizeof(data));
    uint8_t CMD = 0x00;
    uint8_t CRC1 = ((uint16_t)CRC) & 0xFFu;
    uint8_t CRC2 = (CRC & 0xFF00u) >> 8;
    uint8_t W_Frame[] = {0xABu, 0xCDu, CMD, sizeof(data), data[0], 0xE1u, 0xE2u, CRC1, CRC2};

    // Serial2.write(W_Frame, sizeof(W_Frame));
    // uint8_t R_Frame[] = {0xABu, 0xCDu, CMD | 0x80u, 0, 0xE1u, 0xE2u};
    // send dev_0
    // uart_write_bytes(uart_num, (const char *)R_Frame, sizeof(R_Frame));

    // send dev1
    // R_Frame[2] |= 1 << 0;
    // uart_write_bytes(uart_num, (const char *)R_Frame, sizeof(R_Frame));
    uart_write_bytes(uart_num, (const char *)W_Frame, 9);

    // reset
    needToUpdate = 0;
    // pCharacteristic[DEV_1_ID]->setValue(&startToCalculate, 1);
  }
  // Serial.printf("\nLedI = %d", ledBufIndex);
  delay(10);
}