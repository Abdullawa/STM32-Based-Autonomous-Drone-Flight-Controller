#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

void Telemetry_Init(void);

void Telemetry_Send(float pitch, float roll,
                    float pitch_rate, float roll_rate,
                    float yaw_rate,
                    uint16_t throttle);

void Telemetry_ReceiveCheck(void);
void Telemetry_TxCpltCallback(void);

#endif // TELEMETRY_H