#include <RF24.h>
#include <HardwareSerial.h>

RF24 radio(10, 11);

struct __attribute__((packed)) dataPacket {
  int16_t pitch;
  int16_t roll;
  int16_t yaw;
  int16_t throttle;
  uint8_t sync;     // 0xAA
  uint8_t type;     // 0x01
  uint8_t kill;     // 0 or 1
  uint8_t seq;      // sequence number (wraps at 255)
};

static uint8_t tx_seq = 0;

int16_t readThrottle(){
  return ((analogRead(5) - 1570) * 16) / 10;
}

int16_t readYaw(){
  return ((analogRead(4) - 1646) * 16) / 10;
}

int16_t readPitch(){
  return ((analogRead(0) - 1534) * 16) / 10;
}

int16_t readRoll(){
  return ((analogRead(1) - 1594) * 16) / 10;
}

dataPacket packetToSend(int16_t thr, int16_t yw, int16_t pt, int16_t rl, uint8_t killFlag){
  dataPacket packet;
  packet.pitch    = pt;
  packet.roll     = rl;
  packet.yaw      = yw;
  packet.throttle = thr;
  packet.sync     = 0xAA;
  packet.type     = 0x01;
  packet.kill     = killFlag ? 1 : 0;
  packet.seq      = tx_seq++;
  return packet;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  SPI.begin(6, 2, 7, 11);

  if (!radio.begin(&SPI)) {
    Serial.println("NRF24 hardware not responding!");
    while (1) {}
  }

  radio.setChannel(100);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setCRCLength(RF24_CRC_16);   // add this
  radio.setPayloadSize(sizeof(dataPacket));
  radio.openWritingPipe(0xE7E7E7E7E7LL);
  radio.stopListening();
  radio.printDetails();
}

void loop() {
  dataPacket packet = packetToSend(readThrottle(), readYaw(), readPitch(), readRoll(), 0);
  radio.write(&packet, sizeof(packet));

  char data[128];
  snprintf(data, sizeof(data), "seq:%u throttle:%d pitch:%d roll:%d yaw:%d kill:%u\n",
           packet.seq, packet.throttle, packet.pitch, packet.roll, packet.yaw, packet.kill);
  Serial.print(data);

  delay(20); // 50Hz
}