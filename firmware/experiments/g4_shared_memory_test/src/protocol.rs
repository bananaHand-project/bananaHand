use cobs::{decode, encode};

pub const COBS_DELIM: u8 = 0x00;
pub const MAX_FRAME: usize = 128;
pub const PAYLOAD_LEN: usize = 20;

fn checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

pub struct BuiltFrame {
    pub buf: [u8; MAX_FRAME],
    pub len: usize,
}

pub fn build_frame(msg_type: u8, readings: [u16; 10]) -> BuiltFrame {
    let mut body = [0u8; MAX_FRAME];
    let mut body_len = 0usize;

    body[body_len] = msg_type;
    body_len += 1;

    let payload_start = body_len;

    for reading in readings {
        let bytes = reading.to_le_bytes();
        body[body_len..body_len + 2].copy_from_slice(&bytes);
        body_len += 2;
    }

    let chk = checksum(&body[payload_start..payload_start + PAYLOAD_LEN]);
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

pub enum MessageType {
    ForceReadings = 0x01,
}

pub struct FrameParser {
    enc_buf: [u8; MAX_FRAME],
    enc_len: usize,
    dec_buf: [u8; MAX_FRAME],
}

impl FrameParser {
    pub fn new() -> Self {
        Self {
            enc_buf: [0; MAX_FRAME],
            enc_len: 0,
            dec_buf: [0; MAX_FRAME],
        }
    }

    pub fn parse_byte(&mut self, byte: u8) -> Option<(u8, &[u8])> {
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

        if self.enc_len == 0 {
            return None;
        }

        let report = match decode(&self.enc_buf[..self.enc_len], &mut self.dec_buf) {
            Ok(r) => r,
            Err(_) => {
                defmt::error!("COBS decode failed");
                self.enc_len = 0;
                return None;
            }
        };

        let dec_len = report.frame_size();
        self.enc_len = 0;

        if dec_len < 3 {
            defmt::error!("Frame too short");
            return None;
        }

        let msg_type = self.dec_buf[0];

        if dec_len != 1 + PAYLOAD_LEN + 1 {
            defmt::error!("Length mismatch");
            return None;
        }

        let payload_start = 1;
        let payload_end = payload_start + PAYLOAD_LEN;
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
