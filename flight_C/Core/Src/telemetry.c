#include "telemetry.h"
#include "nrf24l01p.h"
#include <math.h>
#include <stdint.h>

// =====================================================
// PACKET (MUST MATCH ESP32 EXACTLY)
// =====================================================
#pragma pack(push, 1)
typedef struct
{
    uint8_t sync;
    uint8_t type;

    int16_t pitch;
    int16_t roll;

    int16_t pitch_rate;
    int16_t roll_rate;
    int16_t yaw_rate;

    uint16_t throttle;
} TelemetryPacket;
#pragma pack(pop)

// =====================================================
static uint32_t telem_last = 0;

// =====================================================
void Telemetry_Init(void)
{
    telem_last = 0;
    nrf24l01p_tx_init(100, _1Mbps);
}

// =====================================================
void Telemetry_Send(float pitch, float roll,
                    float pitch_rate, float roll_rate,
                    float yaw_rate,
                    uint16_t throttle)
{
    if ((HAL_GetTick() - telem_last) < 50)
        return;

    telem_last = HAL_GetTick();

    TelemetryPacket pkt;

    pkt.sync = 0xAA;
    pkt.type = 0x04;

    pkt.pitch = (int16_t)(pitch * 100.0f);
    pkt.roll  = (int16_t)(roll * 100.0f);

    pkt.pitch_rate = (int16_t)(pitch_rate * 100.0f);
    pkt.roll_rate  = (int16_t)(roll_rate * 100.0f);
    pkt.yaw_rate   = (int16_t)(yaw_rate * 100.0f);

    pkt.throttle = throttle;

    HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);

    nrf24l01p_tx_transmit((uint8_t*)&pkt);
}

// =====================================================
void Telemetry_ReceiveCheck(void)
{
    uint8_t status = nrf24l01p_get_fifo_status();

    if ((status & 0x01) == 0)
    {
        uint8_t buf[32];
        nrf24l01p_rx_receive(buf);

        TelemetryPacket *pkt = (TelemetryPacket*)buf;

        if (pkt->sync == 0xAA && pkt->type == 0x04)
        {
            // valid packet
        }
    }
}

// =====================================================
void Telemetry_TxCpltCallback(void)
{
}