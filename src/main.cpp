#include <Arduino.h>
#include <ACAN_ESP32.h>

SemaphoreHandle_t mySemaphore;

#define SERIAL_RX_BUFFER_SIZE 1024


unsigned long previousMillis = 0; // will store last time a CAN Message was send
const int interval = 1000;        // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;     // Receive Queue size
void can_managment_task(void *pvParameters);
void uart_managment_task(void *pvParameters);
void uart_rx();
uint16_t cobs_encode(const void *data, uint16_t length, uint8_t *buffer);
uint16_t cobs_decode(const uint8_t *buffer, size_t length, void *data);
void send_test_message();

static const uint32_t DESIRED_BIT_RATE = 1000UL * 1000UL;

typedef struct
{
  uint8_t id;
  uint8_t data[8];
  uint8_t len;
} CanMessage;

#define QUEUE_LENGTH 20
#define QUEUE_ITEM_SIZE sizeof(CanMessage)

#define MAX_BUFFER_SIZE 200

uint8_t uart_rx_buffer[MAX_BUFFER_SIZE];
uint8_t uart_rx_cobs_buffer[MAX_BUFFER_SIZE];
uint8_t uart_rx_buffer_index = 0;

uint8_t uart_tx_buffer[MAX_BUFFER_SIZE];
uint8_t uart_tx_cobs_buffer[MAX_BUFFER_SIZE];
uint8_t uart_tx_buffer_index = 0;

uint8_t previous_bytes[3] = {1, 1, 1};
uint8_t counter = 0;

bool ledState = LOW;

xQueueHandle canTX_queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
xQueueHandle canRX_queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);

void setup()
{
  Serial.begin(3000000);
  delay(100);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");
  Serial.onReceive(uart_rx, false);
  // Wire.begin();

  pinMode(16, OUTPUT);

  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode;
  settings.mRxPin = GPIO_NUM_26;
  settings.mTxPin = GPIO_NUM_25;
  const ACAN_ESP32_Filter filter = ACAN_ESP32_Filter::singleStandardFilter(ACAN_ESP32_Filter::data, 0xfff, 0xfff);

  ACAN_ESP32::can.begin(settings, filter);
  // setup serial event callback function

  // xTaskCreatePinnedToCore(can_managment_task, "can_managment_task", 40000, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(uart_managment_task, "uart_managment_task", 40000, NULL, 0, NULL, 0);
  // while (1){
  //   send_test_message();
  //   delay(100);
  // }
}

void send_test_message()
{
  CANMessage frame;
  frame.ext = false;
  frame.id = 02;
  frame.len = 2;
  frame.data[0] = 0x01;
  frame.data[1] = 0x02;
  ACAN_ESP32::can.tryToSend(frame);
}

void loop()
{
  CANMessage frame;
  if (uart_rx_buffer_index > 2 && uart_rx_cobs_buffer[0] == 0 && uart_rx_cobs_buffer[uart_rx_buffer_index - 1] == 0)
  {
    cobs_decode(uart_rx_cobs_buffer + 1, uart_rx_buffer_index - 2, uart_rx_buffer);

    uart_rx_buffer_index = 0;
    uint8_t num_messages = uart_rx_buffer[0];
    uint16_t previous_offset = 1;
    for (int i = 0; i < num_messages; i++)
    {
      CanMessage ToCanTx;
      ToCanTx.id = uart_rx_buffer[i + previous_offset];
      ToCanTx.len = uart_rx_buffer[i + previous_offset + 1];
      for (int j = 0; j < ToCanTx.len; j++)
      {
        ToCanTx.data[j] = uart_rx_buffer[i + previous_offset + 2 + j];
      }
      previous_offset += ToCanTx.len + 1;
      frame.ext = false;
      frame.id = ToCanTx.id;
      frame.len = ToCanTx.len;
      for (int i = 0; i < ToCanTx.len; i++)
      {
        frame.data[i] = ToCanTx.data[i];
      }
      ACAN_ESP32::can.tryToSend(frame);
    }
  }
}

void uart_managment_task(void *pvParameters)
{
  CANMessage frame;
  while (1)
  {
    if (ACAN_ESP32::can.receive(frame))
    {
      uart_tx_buffer[0] = frame.id;
      uart_tx_buffer[1] = frame.len;
      for (int i = 0; i < frame.len; i++)
      {
        uart_tx_buffer[i + 2] = frame.data[i];
      }
      uint16_t cobs_len = cobs_encode(uart_tx_buffer, frame.len + 2, uart_tx_cobs_buffer);
      Serial.write(0);
      for (int i = 0; i < cobs_len; i++)
      {
        Serial.write(uart_tx_cobs_buffer[i]);
        // No DMA :(
      }
      Serial.write(0);
    }
  }
}

uint16_t cobs_encode(const void *data, uint16_t length, uint8_t *buffer)
{

  uint8_t *encode = buffer;  // Encoded byte pointer
  uint8_t *codep = encode++; // Output code pointer
  uint8_t code = 1;          // Code value

  for (const uint8_t *byte = (const uint8_t *)data; length--; ++byte)
  {
    if (*byte)
    { // Byte not zero, write it
      *encode++ = *byte, ++code;
    }

    if (!*byte || code == 0xff) // Input is zero or block completed, restart
    {
      *codep = code, code = 1, codep = encode;
      if (!*byte || length)
      {
        ++encode;
      }
    }
  }
  *codep = code; // Write final code value

  return (encode - buffer);
}

uint16_t cobs_decode(const uint8_t *buffer, size_t length, void *data)
{
  const uint8_t *byte = buffer;      // Encoded input byte pointer
  uint8_t *decode = (uint8_t *)data; // Decoded output byte pointer

  for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block)
  {
    if (block)
    { // Decode block byte
      *decode++ = *byte++;
    }
    else
    {
      block = *byte++; // Fetch the next block length
      if (block &&
          (code != 0xff))
      { // Encoded zero, write it unless it's delimiter.
        *decode++ = 0;
      }
      code = block;
      if (!code)
      { // Delimiter code found
        break;
      }
    }
  }

  return (decode - (uint8_t *)data);
}

void update_previous_bytes(uint8_t *previous_bytes, uint8_t new_byte)
{
  previous_bytes[0] = previous_bytes[1];
  previous_bytes[1] = previous_bytes[2];
  previous_bytes[2] = new_byte;
}

uint8_t check_all_previous_bytes_zero(uint8_t *previous_bytes)
{
  if (previous_bytes[0] == 0 && previous_bytes[1] == 0 && previous_bytes[2] == 0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void uart_rx()
{
  // while(Serial.available()){
  //   Serial.print(Serial.read(), HEX);
  // }
  uint8_t buffer[1];
  while (Serial.available())
  {
    digitalWrite(16, ledState);
    ledState = !ledState;
    Serial.readBytes(buffer, 1);
    // Serial.print(buffer[0], HEX);

    uart_rx_cobs_buffer[uart_rx_buffer_index] = buffer[0];
    uart_rx_buffer_index++;
  }

  update_previous_bytes(previous_bytes, uart_rx_cobs_buffer[uart_rx_buffer_index]);

  if (uart_rx_cobs_buffer[0] != 0)
  {
    uart_rx_buffer_index = 0;
  }
}
