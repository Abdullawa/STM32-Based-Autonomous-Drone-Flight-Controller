#include "receiver.h"
#include "nrf24l01p.h"
#include <string.h>

// Simple circular buffer to hold a few incoming packets
#define RX_BUFFER_SIZE 8

static dataPacket rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;
static volatile bool irq_flag = false;
static volatile bool overflow_flag = false;

void Receiver_Init(void)
{
  // Use nRF channel index 100 to match transmitter
  nrf24l01p_rx_init(100, _1Mbps);
}

void Receiver_OnIrq(void)
{
  // Defer heavy work to main context
  irq_flag = true;
}

bool Receiver_IsIrqPin(uint16_t GPIO_Pin)
{
  return GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER;
}

// Call from main loop to service IRQs and read payloads via SPI (safe context)
void Receiver_Process(void)
{
  if (!irq_flag) return;

  __disable_irq();
  irq_flag = false;
  __enable_irq();

  uint8_t payload[NRF24L01P_PAYLOAD_LENGTH];

  // Read until RX FIFO empty. RX_EMPTY bit (bit0) == 1 when empty for nRF24
  while ((nrf24l01p_get_fifo_status() & 0x01) == 0)
  {
    // Perform blocking SPI read now that we're outside ISR
    nrf24l01p_rx_receive(payload);

    uint8_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
    if (next_head == rx_tail)
    {
      // buffer overflow: drop oldest
      overflow_flag = true;
      rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    }

    // Copy payload into ring buffer
    memcpy(&rx_buffer[rx_head], payload, sizeof(dataPacket));
    rx_head = next_head;
  }
}

dataPacket Receiver_GetPacket(void)
{
  dataPacket empty = {0};
  return empty;
}

bool Receiver_ReadPacket(dataPacket *packet)
{
  // Check empty
  if (rx_head == rx_tail) return false;

  __disable_irq();
  *packet = rx_buffer[rx_tail];
  rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
  __enable_irq();

  return true;
}
