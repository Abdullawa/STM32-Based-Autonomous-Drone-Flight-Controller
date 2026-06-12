#include <SPI.h>
#include <RF24.h>

RF24 radio(10, 11);

#pragma pack(push, 1)
struct TelemetryPacket
{
    uint8_t sync;        // 0xAA
    uint8_t type;        // 0x04

    int16_t pitch;       
    int16_t roll;        

    int16_t pitch_rate;  
    int16_t roll_rate;   

    int16_t yaw_rate;    // IMPORTANT: matches updated STM32

    uint16_t throttle;   // was pwm_fl on STM32 side
};
#pragma pack(pop)

void setup()
{
    Serial.begin(115200);
    delay(500);

    SPI.begin(6, 2, 7, 11);

    if (!radio.begin(&SPI))
    {
        Serial.println("FAIL: Radio not responding");
        while (1) {}
    }

    radio.setChannel(100);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setAutoAck(false);
    radio.setCRCLength(RF24_CRC_16);

    // MUST match STM32 payload size
    radio.setPayloadSize(sizeof(TelemetryPacket));

    // Address must match STM32
    radio.openReadingPipe(0, 0xE7E7E7E7E7LL);
    radio.startListening();

    Serial.println("Waiting for packets...");
}

void loop()
{
    TelemetryPacket pkt;

    if (radio.available())
    {
        radio.read(&pkt, sizeof(pkt));

        // Sync check (VERY IMPORTANT for noise rejection)
        if (pkt.sync != 0xAA)
            return;

        Serial.printf(
            "P=%.2f R=%.2f PR=%.2f RR=%.2f YR=%.2f TH=%u\n",
            pkt.pitch / 100.0f,
            pkt.roll / 100.0f,
            pkt.pitch_rate / 100.0f,
            pkt.roll_rate / 100.0f,
            pkt.yaw_rate / 100.0f,
            pkt.throttle
        );
    }

    delay(10);
}