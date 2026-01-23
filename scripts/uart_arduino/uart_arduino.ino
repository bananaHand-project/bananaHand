#include <Arduino.h>

// ======================
// Protocol constants
// ======================
const uint8_t START_BYTE = 0xFF;
const uint8_t END_BYTE   = 0xFE;
const uint8_t MSG_TYPE_POSITION = 0x01;

// ======================
// Checksum = sum(payload) & 0xFF
// ======================
uint8_t checksum(const uint8_t* data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += data[i];
  return sum & 0xFF;
}

// ======================
// Float <-> Bytes
// ======================
void floatToBytes(float f, uint8_t out[4]) { memcpy(out, &f, 4); }
float bytesToFloat(const uint8_t b[4]) { float f; memcpy(&f, b, 4); return f; }

// ======================
// Build frame like Python build_frame()
// ======================
size_t buildFrame(const float* positions, uint8_t* out) {
  size_t idx = 0;

  out[idx++] = START_BYTE;
  out[idx++] = MSG_TYPE_POSITION;

  const uint8_t payloadLen = 8 * 4;
  out[idx++] = payloadLen;

  // Payload floats
  for (int i = 0; i < 8; i++) {
    uint8_t b[4];
    floatToBytes(positions[i], b);
    out[idx++] = b[0];
    out[idx++] = b[1];
    out[idx++] = b[2];
    out[idx++] = b[3];
  }

  // Checksum
  uint8_t chk = checksum(&out[3], payloadLen);
  out[idx++] = chk;

  // End byte
  out[idx++] = END_BYTE;

  return idx;
}

// ======================
// Blocking read N bytes (with timeout)
// ======================
bool readBytesTimeout(uint8_t* buf, size_t len, unsigned long timeoutMs = 200) {
  unsigned long start = millis();
  size_t count = 0;
  while (count < len) {
    if (Serial.available()) buf[count++] = Serial.read();
    if (millis() - start > timeoutMs) return false;
  }
  return true;
}

// ======================
// Parse received frame
// ======================
bool parseFrame(const uint8_t* frame, size_t len, float* outPositions) {
  if (len < 6) return false;
  if (frame[0] != START_BYTE) return false;
  if (frame[1] != MSG_TYPE_POSITION) return false;

  uint8_t payloadLen = frame[2];
  if (payloadLen != 32) return false;

  if (frame[len - 1] != END_BYTE) return false;

  uint8_t chk = frame[3 + payloadLen];
  if (checksum(&frame[3], payloadLen) != chk) {
    Serial.println("Checksum mismatch");
    return false;
  }

  for (int i = 0; i < 8; i++) {
    outPositions[i] = bytesToFloat(&frame[3 + i * 4]);
  }

  return true;
}

// ======================
// Setup
// ======================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Arduino TX/RX protocol driver ready.");
}

// ======================
// Main Loop: Arduino acts like Python
// ======================
void loop() {
  // 1️⃣ Create 8 actuator positions (demo: random or fixed)
  float positions[8];
  for (int i = 0; i < 8; i++) {
    positions[i] = random(100, 2000) / 100.0;  // 1.00 to 20.00
  }

  // 2️⃣ Build frame
  uint8_t frame[128];
  size_t frameLen = buildFrame(positions, frame);

  unsigned long sendTime = micros();

  // 3️⃣ Send frame
  Serial.write(frame, frameLen);
  Serial.flush();

  // 4️⃣ Read response header
  uint8_t hdr[3];
  if (!readBytesTimeout(hdr, 3, 500)) {
    Serial.println("Timeout: no response header");
    return;
  }

  if (hdr[0] != START_BYTE) {
    Serial.println("Invalid header start, draining");
    while (Serial.available()) Serial.read();
    return;
  }

  uint8_t respType = hdr[1];
  uint8_t respLen  = hdr[2];

  // 5️⃣ Read rest of frame
  uint8_t rest[64];
  if (!readBytesTimeout(rest, respLen + 2, 200)) {
    Serial.println("Incomplete response");
    return;
  }

  // 6️⃣ Combine full frame
  uint8_t rxFrame[128];
  rxFrame[0] = hdr[0];
  rxFrame[1] = hdr[1];
  rxFrame[2] = hdr[2];
  memcpy(&rxFrame[3], rest, respLen + 2);

  float respPositions[8];
  if (parseFrame(rxFrame, respLen + 5, respPositions)) {
    unsigned long recvTime = micros();
    float rtt = (recvTime - sendTime) / 1000.0;  // ms

    Serial.print("RTT = ");
    Serial.print(rtt, 2);
    Serial.print(" ms | sent: ");

    for (int i = 0; i < 8; i++) {
      Serial.print(positions[i], 3);
      Serial.print(" ");
    }

    Serial.print("| recv: ");
    for (int i = 0; i < 8; i++) {
      Serial.print(respPositions[i], 3);
      Serial.print(" ");
    }
    Serial.println();
  }

  delay(20);  // small pacing delay
}
