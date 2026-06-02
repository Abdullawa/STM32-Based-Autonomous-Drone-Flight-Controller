#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdint.h>
#include <stdbool.h>

// 12-byte packed packet matching transmitter: 8 bytes of int16 + 4 bytes of u8 fields
typedef struct __attribute__((packed))
{
  int16_t pitch;
  int16_t roll;
  int16_t yaw;
  int16_t throttle;
  uint8_t sync;
  uint8_t type;
  uint8_t kill;
  uint8_t seq;
} dataPacket;

void Receiver_Init(void);
void Receiver_OnIrq(void);
bool Receiver_IsIrqPin(uint16_t GPIO_Pin);
bool Receiver_ReadPacket(dataPacket *packet);
dataPacket Receiver_GetPacket(void);

// Process pending RX events (deferred from IRQ). Call from main loop.
void Receiver_Process(void);

#endif
