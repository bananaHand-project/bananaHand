#include <Arduino.h>

// Protocol constants
const uint8_t START_BYTE = 0xFF;
const uint8_t END_BYTE   = 0xFE;
const uint8_t MSG_TYPE_POSITION = 0x01;

// --- Checksum (same as Python) ---
uint8_t checksum(const uint8_t* data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFF;
}

// --- Convert 4 bytes → float (little endian) ---
float bytesToFloat(const uint8_t b[4]) {
  float f;
  memcpy(&f, b, 4);
  return f;
}

// --- Convert float → 4 bytes (little endian) ---
void floatToBytes(float f, uint8_t out[4]) {
  memcpy(out, &f, 4);
}

// --- Read N bytes with blocking timeout ---
bool readBytesTimeout(uint8_t* buf, size_t len, unsigned long timeoutMs = 200) {
  unsigned long start = millis();
  size_t count = 0;
  while (count < len) {
    if (Serial.available()) {
      buf[count++] = Serial.read();
    }
    if (millis() - start > timeoutMs) {
      return false;
    }
  }
  return true;
}

// -----------------------------
// Parse frame coming from Python
// -----------------------------
bool parseFrame(uint8_t* frame, size_t frame_len, float positions_out[8]) {
  if (frame_len < 6) return false;
  if (frame[0] != START_BYTE) return false;
  if (frame[1] != MSG_TYPE_POSITION) return false;

  uint8_t len = frame[2];
  if (frame[frame_len - 1] != END_BYTE) return false;

  uint8_t chk = frame[3 + len];
  if (checksum(&frame[3], len) != chk) {
    Serial.println("Checksum mismatch");
    return false;
  }

  // Extract 8 floats (32 bytes)
  if (len != 32) return false;

  for (int i = 0; i < 8; i++) {
    positions_out[i] = bytesToFloat(&frame[3 + i * 4]);
  }

  return true;
}

// -----------------------------
// Build echo frame back to Python
// -----------------------------
size_t buildFrame(const float* positions, uint8_t* outFrame) {
  size_t idx = 0;

  outFrame[idx++] = START_BYTE;
  outFrame[idx++] = MSG_TYPE_POSITION;
  outFrame[idx++] = 8 * 4; // payload length = 32 bytes

  // Payload
  for (int i = 0; i < 8; i++) {
    uint8_t b[4];
    floatToBytes(positions[i], b);
    outFrame[idx++] = b[0];
    outFrame[idx++] = b[1];
    outFrame[idx++] = b[2];
    outFrame[idx++] = b[3];
  }

  uint8_t chk = checksum(&outFrame[3], 32);
  outFrame[idx++] = chk;

  outFrame[idx++] = END_BYTE;

  return idx;  // total frame length
}

// =====================================================
//                      SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Arduino binary protocol ready.");
}

// =====================================================
//                      LOOP
// =====================================================
void loop() {
  if (!Serial.available()) return;

  // Read header: [FF][TYPE][LEN]
  uint8_t header[3];
  if (!readBytesTimeout(header, 3)) return;

  if (header[0] != START_BYTE) {
    // Drain garbage
    while (Serial.available()) Serial.read();
    return;
  }

  uint8_t msg_type = header[1];
  uint8_t length   = header[2];

  // Read the rest: payload + checksum + end byte
  uint8_t rest[64];
  size_t expected = length + 2;  // [payload] + [chk] + [END]
  if (!readBytesTimeout(rest, expected)) return;

  // Combine frame
  uint8_t frame[128];
  frame[0] = header[0];
  frame[1] = header[1];
  frame[2] = header[2];
  memcpy(&frame[3], rest, expected);

  float positions[8];
  if (parseFrame(frame, 3 + expected, positions)) {
    Serial.print("Parsed floats: ");
    for (int i = 0; i < 8; i++) {
      Serial.print(positions[i], 3);
      Serial.print(" ");
    }
    Serial.println();

    // --- ECHO frame back ---
    uint8_t outFrame[128];
    size_t outLen = buildFrame(positions, outFrame);
    Serial.write(outFrame, outLen);
  }
}
