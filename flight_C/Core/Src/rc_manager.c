#include "rc_manager.h"
#include <stdio.h>

#define LINK_LOSS_TIMEOUT_MS 200

static uint32_t last_packet_ms  = 0;
static bool     latched_kill    = false;
static uint8_t  last_seq        = 0;
static bool     seq_initialized = false;
static dataPacket latest_packet = {0};
static bool     has_packet      = false;

void RC_Update(void)
{
    dataPacket controller;

    if (Receiver_ReadPacket(&controller) && controller.sync == 0xAA && controller.type == 0x01)
    {
        if (!seq_initialized || controller.seq != last_seq)
        {
            seq_initialized = true;
            last_seq        = controller.seq;
        }

        last_packet_ms = HAL_GetTick();
        latest_packet  = controller;
        has_packet     = true;

        if (controller.kill)
            latched_kill = true;

        HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
    }

    // link loss failsafe
    if ((HAL_GetTick() - last_packet_ms) > LINK_LOSS_TIMEOUT_MS)
        latched_kill = true;

    // LED reflects kill state
    if (latched_kill)
        HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
}

void RC_PrintTelemetry(void)
{
    if (!has_packet) return;

    static dataPacket last_printed = {0};

    // Only print if something changed
    if (latest_packet.pitch    == last_printed.pitch    &&
        latest_packet.roll     == last_printed.roll     &&
        latest_packet.yaw      == last_printed.yaw      &&
        latest_packet.throttle == last_printed.throttle &&
        latest_packet.kill     == last_printed.kill)
        return;

    last_printed = latest_packet;

    char msg[128];
    int len = snprintf(msg, sizeof(msg),
                       "Remote: seq:%u pitch:%d roll:%d yaw:%d throttle:%d kill:%u\r\n",
                       latest_packet.seq,
                       latest_packet.pitch,
                       latest_packet.roll,
                       latest_packet.yaw,
                       latest_packet.throttle,
                       latest_packet.kill);

    HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, 100);
}

bool RC_GetLatestPacket(dataPacket *out)
{
    if (!has_packet) return false;
    *out = latest_packet;
    return true;
}

bool RC_IsKillActive(void)
{
    return latched_kill;
}