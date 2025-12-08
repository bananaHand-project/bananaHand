use heapless::CapacityError;

pub const START_BYTE: u8 = 0xFF;
pub const END_BYTE: u8 = 0xFE;

pub fn checksum(data: &[u8]) -> u8 {
    let sum: u8 = data.iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
    sum
}

pub fn build_frame(msg_type: u8, positions: [f32; 8]) -> heapless::Vec<u8, 128> {
    let mut frame = heapless::Vec::<u8, 128>::new();

    let _ = frame.push(START_BYTE);
    let _ = frame.push(msg_type);
    let len = positions.len() * 4; // each f32 is 4 bytes
    let _ = frame.push(len as u8);

    let mut pos_status: Result<(), CapacityError> = Ok(());
    for pos in positions {
        pos_status = frame.extend_from_slice(&pos.to_le_bytes());
    }
    if pos_status.is_err() {
        defmt::error!("Failed to build frame: CapacityError");
    }

    let _  = frame.push(checksum(&frame.as_slice()[3..(len + 3)]));
    let _  = frame.push(END_BYTE);

    frame
}

pub enum MessageType {
    PositionUpdate = 0x01,
}

pub enum ParseState {
    WaitingForStart,
    ReadingType,
    ReadingLength { msg_type: u8 },
    ReadingData { msg_type: u8, length: usize },
    ReadingChecksum { msg_type: u8, length: usize },
    WaitingForEnd { msg_type: u8, length: usize, checksum_byte: u8 },
}

pub struct FrameParser {
    state: ParseState,
    pub buf: heapless::Vec<u8, 128>,
}

impl FrameParser {
    pub fn new() -> Self {
        FrameParser {
            state: ParseState::WaitingForStart,
            buf: heapless::Vec::new(),
        }
    }

    pub fn parse_byte(&mut self, byte: u8) -> Option<(u8, heapless::Vec<u8, 128>)> {
        // take ownership of the current state so we can reassign self.state freely
        let mut state = core::mem::replace(&mut self.state, ParseState::WaitingForStart);
        let mut result: Option<(u8, heapless::Vec<u8, 128>)> = None;

        match state {
            ParseState::WaitingForStart => {
                if byte == START_BYTE {
                    // prepare buffer for incoming payload
                    let _ = self.buf.clear();
                    state = ParseState::ReadingType;
                } else {
                    state = ParseState::WaitingForStart;
                }
            }
            ParseState::ReadingType => {
                let msg_type = byte;
                state = ParseState::ReadingLength { msg_type };
            }
            ParseState::ReadingLength { msg_type } => {
                let length = byte as usize;
                // clear buffer and start collecting payload bytes into self.buf
                let _ = self.buf.clear();
                state = ParseState::ReadingData { msg_type, length };
            }
            ParseState::ReadingData { msg_type, length } => {
                let _ = self.buf.push(byte);
                if self.buf.len() == length {
                    state = ParseState::ReadingChecksum { msg_type, length };
                } else {
                    state = ParseState::ReadingData { msg_type, length };
                }
            }
            ParseState::ReadingChecksum { msg_type, length } => {
                let checksum_byte = byte;
                state = ParseState::WaitingForEnd { msg_type, length, checksum_byte };
            }
            ParseState::WaitingForEnd { msg_type, length: _length, checksum_byte } => {
                if byte == END_BYTE {
                    let calculated_checksum = checksum(&self.buf);
                    if checksum_byte == calculated_checksum {
                        // return a clone of the internal buffer so caller can use it
                        result = Some((msg_type, self.buf.clone()));
                        // clear buffer for next frame
                        let _ = self.buf.clear();
                        state = ParseState::WaitingForStart;
                    } else {
                        defmt::error!("Checksum mismatch: expected {}, got {}", calculated_checksum, checksum_byte);
                        let _ = self.buf.clear();
                        state = ParseState::WaitingForStart;
                    }
                } else {
                    // unexpected byte, reset parser
                    let _ = self.buf.clear();
                    state = ParseState::WaitingForStart;
                }
            }
        }

        // store the updated state back
        self.state = state;
        result
    }
}

