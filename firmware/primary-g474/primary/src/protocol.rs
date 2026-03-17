use cobs::{decode, encode};

pub const COBS_DELIM: u8 = 0x00;
pub const MAX_FRAME: usize = 128;

pub enum MessageType {
    PositionUpdate = 0x01,
    ForceReadings = 0x02,
    TelemetryCombined = 0x03,
}

fn checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

pub struct BuiltFrame {
    pub buf: [u8; MAX_FRAME],
    pub len: usize,
}

pub fn build_frame<const N: usize>(msg_type: u8, values: [u16; N]) -> BuiltFrame {
    const fn payload_len(n: usize) -> usize {
        n * 2
    }

    let payload_len = payload_len(N);

    let mut body = [0u8; MAX_FRAME];
    let mut body_len = 0usize;

    // body: [type][payload...][chk]
    body[body_len] = msg_type;
    body_len += 1;

    let payload_start = body_len;

    for v in values {
        let bytes = v.to_le_bytes();
        body[body_len..body_len + 2].copy_from_slice(&bytes);
        body_len += 2;
    }

    let chk = checksum(&body[payload_start..payload_start + payload_len]);
    body[body_len] = chk;
    body_len += 1;

    // ---- COBS encode + delimiter ----
    let mut framed = BuiltFrame {
        buf: [0u8; MAX_FRAME],
        len: 0,
    };

    let enc_len = encode(&body[..body_len], &mut framed.buf);

    // ensure space for delimiter
    if enc_len + 1 > MAX_FRAME {
        defmt::error!("COBS encode overflow: no room for delimiter");
        framed.len = 0;
        return framed;
    }

    framed.buf[enc_len] = COBS_DELIM;
    framed.len = enc_len + 1;

    framed
}

pub fn build_telemetry_frame<const P: usize, const F: usize>(
    positions: [u16; P],
    forces: [u16; F],
) -> BuiltFrame {
    let payload_len = (P + F) * 2;

    let mut body = [0u8; MAX_FRAME];
    let mut body_len = 0usize;

    // body: [type][positions...][forces...][chk]
    body[body_len] = MessageType::TelemetryCombined as u8;
    body_len += 1;

    let payload_start = body_len;

    for v in positions {
        let bytes = v.to_le_bytes();
        body[body_len..body_len + 2].copy_from_slice(&bytes);
        body_len += 2;
    }

    for v in forces {
        let bytes = v.to_le_bytes();
        body[body_len..body_len + 2].copy_from_slice(&bytes);
        body_len += 2;
    }

    let chk = checksum(&body[payload_start..payload_start + payload_len]);
    body[body_len] = chk;
    body_len += 1;

    let mut framed = BuiltFrame {
        buf: [0u8; MAX_FRAME],
        len: 0,
    };

    let enc_len = encode(&body[..body_len], &mut framed.buf);
    if enc_len + 1 > MAX_FRAME {
        defmt::error!("COBS encode overflow: no room for delimiter");
        framed.len = 0;
        return framed;
    }

    framed.buf[enc_len] = COBS_DELIM;
    framed.len = enc_len + 1;
    framed
}

pub struct FrameParser<const N: usize> {
    enc_buf: [u8; MAX_FRAME],
    enc_len: usize,
    dec_buf: [u8; MAX_FRAME],
}

impl<const N: usize> FrameParser<N> {
    pub fn new() -> Self {
        Self {
            enc_buf: [0; MAX_FRAME],
            enc_len: 0,
            dec_buf: [0; MAX_FRAME],
        }
    }

    pub fn parse_byte(&mut self, byte: u8) -> Option<(u8, &[u8])> {
        const fn payload_len(n: usize) -> usize {
            n * 2
        }

        let payload_len = payload_len(N);

        // accumulate until delimiter
        if byte != COBS_DELIM {
            if self.enc_len < MAX_FRAME {
                self.enc_buf[self.enc_len] = byte;
                self.enc_len += 1;
            } else {
                defmt::error!("Encoded frame too long, dropping");
                self.enc_len = 0;
            }
            return None;
        }

        // delimiter hit
        if self.enc_len == 0 {
            return None; // ignore empty frame
        }

        let report = match decode(&self.enc_buf[..self.enc_len], &mut self.dec_buf) {
            Ok(r) => r,
            Err(_) => {
                defmt::error!("COBS decode failed");
                self.enc_len = 0;
                return None;
            }
        };

        // cobs 0.5.0: use method, not field
        let dec_len = report.frame_size();
        self.enc_len = 0;

        // decoded body: [type][payload...][chk]
        if dec_len < 3 {
            defmt::error!("Frame too short");
            return None;
        }

        let msg_type = self.dec_buf[0];

        if dec_len != 1 + payload_len + 1 {
            defmt::error!("Length mismatch");
            return None;
        }

        let payload_start = 1;
        let payload_end = payload_start + payload_len;
        let payload = &self.dec_buf[payload_start..payload_end];

        let chk = self.dec_buf[payload_end];
        let calc = checksum(payload);

        if chk != calc {
            defmt::error!("Checksum mismatch");
            return None;
        }

        Some((msg_type, payload))
    }
}
