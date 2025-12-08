import serial
import time
import struct
import random


START_BYTE = 0xFF
END_BYTE = 0xFE
MSG_TYPE_POSITION = 0x01


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def build_frame(positions: list[float]) -> bytes:
    # positions: list of 8 floats
    payload = b"".join(struct.pack("<f", float(p)) for p in positions)
    length = len(payload)
    chk = checksum(payload)
    return bytes([START_BYTE, MSG_TYPE_POSITION, length]) + payload + bytes([chk, END_BYTE])


def parse_frame(frame: bytes) -> list[float] | None:
    if len(frame) < 6:
        return None
    if frame[0] != START_BYTE or frame[-1] != END_BYTE:
        return None
    if frame[1] != MSG_TYPE_POSITION:
        return None
    length = frame[2]
    payload = frame[3:3+length]
    chk = frame[3+length]
    if checksum(payload) != chk:
        print("checksum mismatch")
        return None
    positions = []
    for i in range(0, len(payload), 4):
        if i + 4 > len(payload):
            break
        positions.append(struct.unpack("<f", payload[i:i+4])[0])
    return positions


def main():
    ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=0.2)
    seq = 0
    try:
        while True:
            # generate 8 positions between 1 and 19
            positions = [random.uniform(1.0, 19.0) for _ in range(8)]
            frame = build_frame(positions)

            ser.write(frame)
            try:
                ser.flush()
            except Exception:
                pass

            print(f"sent seq={seq} positions: {[round(x,3) for x in positions]}")
            seq = (seq + 1) & 0xFFFF

            # read header (blocking up to timeout)
            hdr = ser.read(3)
            if len(hdr) < 3:
                print("no response header (timeout)")
                time.sleep(0.1)
                continue

            if hdr[0] != START_BYTE:
                print("invalid start, draining")
                _ = ser.read(256)
                time.sleep(0.1)
                continue

            msg_type = hdr[1]
            length = hdr[2]
            rest = ser.read(length + 2)  # payload + checksum + end
            if len(rest) < (length + 2):
                print("incomplete response")
                time.sleep(0.1)
                continue

            candidate = bytes(hdr + rest)
            positions_resp = parse_frame(candidate)
            if positions_resp is not None:
                print(f"recv seq echo positions: {[round(x,3) for x in positions_resp]}")
            else:
                print(f"invalid frame: {candidate.hex()}")

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("stopping")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
