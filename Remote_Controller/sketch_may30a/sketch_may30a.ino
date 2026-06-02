#include <RF24.h>
#include <HardwareSerial.h>

RF24 radio(10, 11);

// 12-byte packed packet layout:
// int16_t pitch, roll, yaw, throttle = 8 bytes
// uint8_t sync, type, kill, seq = 4 bytes
// total = 12 bytes (matches NRFLite / nRF24 payload length used by receiver)
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

// Read calibrated RC inputs. Keep names distinct from struct fields.
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

dataPacket packetToSend (int16_t thr, int16_t yw, int16_t pt, int16_t rl, uint8_t killFlag){
  dataPacket packet;
  packet.pitch = pt;
  packet.roll = rl;
  packet.yaw = yw;
  packet.throttle = thr;
  packet.sync = 0xAA;
  packet.type = 0x01;
  packet.kill = killFlag ? 1 : 0;
  packet.seq = tx_seq++;
  return packet;
}



void setup() { 
  // put your setup code here, to run once:
  Serial.begin(115200);
  SPI.begin(6, 2, 7, 11);
  radio.begin(&SPI);
  radio.setChannel(100);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  // Reduce retries and delay to lower transmission latency
  // setRetries(delay (250us units), count)
  radio.setRetries(0, 2);
  radio.setPayloadSize(sizeof(dataPacket));
  radio.stopListening();
  radio.printDetails();

}



void loop() {
  // put your main code here, to run repeatedly:
  dataPacket packet;
  packet = packetToSend(readThrottle(), readYaw(), readPitch(), readRoll(), 0);
  bool ok = radio.write(&packet, sizeof(packet));
  (void)ok; // optionally check and retry if false
  char data[128];
  snprintf(data, sizeof(data), "seq:%u throttle:%d pitch:%d roll:%d yaw:%d kill:%u ok:%d\n", packet.seq, packet.throttle, packet.pitch, packet.roll, packet.yaw, packet.kill, ok ? 1 : 0);
  Serial.print(data);
}
