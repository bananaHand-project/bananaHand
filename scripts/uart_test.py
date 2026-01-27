import serial
import time
import struct
import random

from cobs import cobs  # pip install cobs

COBS_DELIM = b"\x00"
MSG_TYPE_POSITION = 0x01

def checksum(data: bytes) -> int:
    return sum(data) & 0xFF

def build_frame_cobs(positions: list[float]) -> bytes:
    # payload: 8 floats, little-endian
    payload = b"".join(struct.pack("<f", float(p)) for p in positions)
    length = len(payload)  # should be 32
    chk = checksum(payload)

    # body: [type][len][payload][chk]
    body = bytes([MSG_TYPE_POSITION, length]) + payload + bytes([chk])

    # on-wire: COBS(body) + 0x00 delimiter
    return cobs.encode(body) + COBS_DELIM

def parse_body(body: bytes) -> list[float] | None:
    # body: [type][len][payload][chk]
    if len(body) < 3:
        return None

    msg_type = body[0]
    if msg_type != MSG_TYPE_POSITION:
        return None

    length = body[1]
    if len(body) != 2 + length + 1:
        # 2 header bytes + payload + 1 checksum
        return None

    payload = body[2:2 + length]
    chk = body[2 + length]

    if checksum(payload) != chk:
        print("checksum mismatch")
        return None

    if length % 4 != 0:
        return None

    positions = []
    for i in range(0, length, 4):
        positions.append(struct.unpack("<f", payload[i:i+4])[0])

    return positions

def read_cobs_frame(ser: serial.Serial, timeout_s: float = 0.2) -> bytes | None:
    """
    Read bytes until a 0x00 delimiter is seen; return the encoded frame (without delimiter).
    Returns None on timeout.
    """
    start = time.monotonic()
    buf = bytearray()

    while True:
        if time.monotonic() - start > timeout_s:
            return None

        chunk = ser.read(64)
        if not chunk:
            continue

        for b in chunk:
            if b == 0:
                # end of frame
                if len(buf) == 0:
                    # ignore empty frame (consecutive delimiters)
                    continue
                return bytes(buf)
            else:
                # cap to avoid runaway buffer if line is noisy
                if len(buf) < 512:
                    buf.append(b)
                else:
                    # drop oversized garbage and resync
                    buf.clear()

def main():
    ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=0.01)

    try:
        while True:
            positions = [random.uniform(1.0, 19.0) for _ in range(8)]
            frame = build_frame_cobs(positions)

            send_time = time.monotonic()
            ser.write(frame)
            try:
                ser.flush()
            except Exception:
                pass

            enc = read_cobs_frame(ser, timeout_s=0.3)
            if enc is None:
                print("no response frame (timeout)")
                continue

            recv_time = time.monotonic()
            rtt_ms = (recv_time - send_time) * 1000.0

            try:
                body = cobs.decode(enc)
            except Exception:
                print(f"COBS decode failed | enc={enc.hex()} | RTT={rtt_ms:.2f} ms")
                continue

            positions_resp = parse_body(body)
            if positions_resp is not None:
                print(
                    f"RTT={rtt_ms:.2f} ms | "
                    f"sent: {[round(x,3) for x in positions]} | "
                    f"recv: {[round(x,3) for x in positions_resp]}"
                )
            else:
                print(f"invalid body: {body.hex()} | RTT={rtt_ms:.2f} ms")

    except KeyboardInterrupt:
        print("stopping")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
