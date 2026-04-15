pub const FORCE_SENSOR_COUNT: usize = 10;
pub const FORCE_BITS_PER_READING: usize = 12;
pub const PACKED_FORCE_DATA_LEN: usize = (FORCE_SENSOR_COUNT * FORCE_BITS_PER_READING) / 8;
pub const FORCE_DATA_PACKET_LEN: usize = PACKED_FORCE_DATA_LEN + 1;
pub const FORCE_MAX_READING: u16 = 4095; // U12 MAX

const CHECKSUM_INDEX: usize = PACKED_FORCE_DATA_LEN;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ForceDataPacket {
    pub index_reading: u16,
    pub middle_reading: u16,
    pub ring_reading: u16,
    pub pinky_reading: u16,
    pub thumb_reading: u16,
    pub palm_1_reading: u16,
    pub palm_2_reading: u16,
    pub palm_3_reading: u16,
    pub palm_4_reading: u16,
    pub palm_5_reading: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ForceDataPacketError {
    InvalidLength { expected: usize, actual: usize },
    ReadingOutOfRange { index: usize, value: u16 },
    ChecksumMismatch { expected: u8, actual: u8 },
}

impl ForceDataPacket {
    pub const LEN: usize = FORCE_DATA_PACKET_LEN;

    pub fn new(readings: [u16; FORCE_SENSOR_COUNT]) -> Result<Self, ForceDataPacketError> {
        validate_readings(&readings)?;
        Ok(Self {
            index_reading: readings[0],
            middle_reading: readings[1],
            ring_reading: readings[2],
            pinky_reading: readings[3],
            thumb_reading: readings[4],
            palm_1_reading: readings[5],
            palm_2_reading: readings[6],
            palm_3_reading: readings[7],
            palm_4_reading: readings[8],
            palm_5_reading: readings[9],
        })
    }

    pub fn readings(&self) -> [u16; FORCE_SENSOR_COUNT] {
        [
            self.index_reading,
            self.middle_reading,
            self.ring_reading,
            self.pinky_reading,
            self.thumb_reading,
            self.palm_1_reading,
            self.palm_2_reading,
            self.palm_3_reading,
            self.palm_4_reading,
            self.palm_5_reading,
        ]
    }

    pub fn into_readings(self) -> [u16; FORCE_SENSOR_COUNT] {
        [
            self.index_reading,
            self.middle_reading,
            self.ring_reading,
            self.pinky_reading,
            self.thumb_reading,
            self.palm_1_reading,
            self.palm_2_reading,
            self.palm_3_reading,
            self.palm_4_reading,
            self.palm_5_reading,
        ]
    }

    pub fn encode(&self) -> Result<[u8; FORCE_DATA_PACKET_LEN], ForceDataPacketError> {
        let mut packet = [0u8; FORCE_DATA_PACKET_LEN];
        self.encode_into(&mut packet)?;
        Ok(packet)
    }

    pub fn encode_into(
        &self,
        packet: &mut [u8; FORCE_DATA_PACKET_LEN],
    ) -> Result<(), ForceDataPacketError> {
        let readings = self.readings();
        validate_readings(&readings)?;
        encode_readings_unchecked(&readings, packet);
        Ok(())
    }

    pub fn parse(packet: &[u8; FORCE_DATA_PACKET_LEN]) -> Result<Self, ForceDataPacketError> {
        Self::parse_slice(packet)
    }

    pub fn parse_slice(packet: &[u8]) -> Result<Self, ForceDataPacketError> {
        if packet.len() != FORCE_DATA_PACKET_LEN {
            return Err(ForceDataPacketError::InvalidLength {
                expected: FORCE_DATA_PACKET_LEN,
                actual: packet.len(),
            });
        }

        let expected_checksum = checksum(&packet[..PACKED_FORCE_DATA_LEN]);
        let actual_checksum = packet[CHECKSUM_INDEX];
        if expected_checksum != actual_checksum {
            return Err(ForceDataPacketError::ChecksumMismatch {
                expected: expected_checksum,
                actual: actual_checksum,
            });
        }

        let mut readings = [0u16; FORCE_SENSOR_COUNT];
        for (pair_idx, chunk) in packet[..PACKED_FORCE_DATA_LEN].chunks_exact(3).enumerate() {
            let a = (chunk[0] as u16) | (((chunk[1] & 0x0F) as u16) << 8);
            let b = ((chunk[1] as u16) >> 4) | ((chunk[2] as u16) << 4);

            readings[pair_idx * 2] = a;
            readings[pair_idx * 2 + 1] = b;
        }

        Self::new(readings)
    }
}

/// Encode sensor readings into the wire packet without validation.
/// Useful since MCU readings ADC values are known to be 12-bit,
/// so callers can avoid per-frame `Result` handling while keeping one packing implementation.
pub fn encode_readings_unchecked(
    readings: &[u16; FORCE_SENSOR_COUNT],
    packet: &mut [u8; FORCE_DATA_PACKET_LEN],
) {
    // Pack each pair of 12-bit readings (a, b) into 3 bytes:
    // byte0 = a[7:0]
    // byte1 = a[11:8] in low nibble | b[3:0] in high nibble
    // byte2 = b[11:4]
    //
    // This yields 24 packed bits for every two 12-bit ADC samples.
    for (pair_idx, chunk) in packet[..PACKED_FORCE_DATA_LEN]
        .chunks_exact_mut(3)
        .enumerate()
    {
        let a = readings[pair_idx * 2] & FORCE_MAX_READING;
        let b = readings[pair_idx * 2 + 1] & FORCE_MAX_READING;

        chunk[0] = (a & 0x00FF) as u8;
        chunk[1] = (((a >> 8) & 0x000F) as u8) | (((b & 0x000F) as u8) << 4);
        chunk[2] = ((b >> 4) & 0x00FF) as u8;
    }

    packet[CHECKSUM_INDEX] = checksum(&packet[..PACKED_FORCE_DATA_LEN]);
}

fn validate_readings(readings: &[u16; FORCE_SENSOR_COUNT]) -> Result<(), ForceDataPacketError> {
    for (index, &value) in readings.iter().enumerate() {
        if value > FORCE_MAX_READING {
            return Err(ForceDataPacketError::ReadingOutOfRange { index, value });
        }
    }
    Ok(())
}

fn checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_preserves_readings() {
        let readings = [0, 1, 15, 16, 255, 256, 1023, 1024, 2047, 4095];
        let packet = ForceDataPacket::new(readings).unwrap().encode().unwrap();
        let parsed = ForceDataPacket::parse(&packet).unwrap();
        assert_eq!(parsed.into_readings(), readings);
    }

    #[test]
    fn packing_layout_is_stable() {
        let packet = ForceDataPacket::new([0xABC, 0x123, 0, 0, 0, 0, 0, 0, 0, 0])
            .unwrap()
            .encode()
            .unwrap();

        assert_eq!(packet[0], 0xBC);
        assert_eq!(packet[1], 0x3A);
        assert_eq!(packet[2], 0x12);
    }

    #[test]
    fn parse_rejects_invalid_checksum() {
        let mut packet = ForceDataPacket::new([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
            .unwrap()
            .encode()
            .unwrap();
        packet[CHECKSUM_INDEX] = packet[CHECKSUM_INDEX].wrapping_add(1);

        assert!(matches!(
            ForceDataPacket::parse(&packet),
            Err(ForceDataPacketError::ChecksumMismatch { .. })
        ));
    }

    #[test]
    fn parse_slice_rejects_invalid_length() {
        let packet = [0u8; FORCE_DATA_PACKET_LEN - 1];

        assert!(matches!(
            ForceDataPacket::parse_slice(&packet),
            Err(ForceDataPacketError::InvalidLength { .. })
        ));
    }

    #[test]
    fn new_rejects_out_of_range_readings() {
        let mut readings = [0u16; FORCE_SENSOR_COUNT];
        readings[4] = FORCE_MAX_READING + 1;

        assert_eq!(
            ForceDataPacket::new(readings),
            Err(ForceDataPacketError::ReadingOutOfRange {
                index: 4,
                value: FORCE_MAX_READING + 1
            })
        );
    }

    #[test]
    fn semantic_fields_map_to_expected_slot_order() {
        let packet = ForceDataPacket {
            index_reading: 11,
            middle_reading: 22,
            ring_reading: 33,
            pinky_reading: 44,
            thumb_reading: 55,
            palm_1_reading: 66,
            palm_2_reading: 77,
            palm_3_reading: 88,
            palm_4_reading: 99,
            palm_5_reading: 111,
        };

        assert_eq!(packet.readings(), [11, 22, 33, 44, 55, 66, 77, 88, 99, 111]);
    }

    #[test]
    fn encode_rejects_out_of_range_when_built_with_literal() {
        let packet = ForceDataPacket {
            index_reading: 0,
            middle_reading: 0,
            ring_reading: 0,
            pinky_reading: 0,
            thumb_reading: FORCE_MAX_READING + 1,
            palm_1_reading: 0,
            palm_2_reading: 0,
            palm_3_reading: 0,
            palm_4_reading: 0,
            palm_5_reading: 0,
        };

        assert_eq!(
            packet.encode(),
            Err(ForceDataPacketError::ReadingOutOfRange {
                index: 4,
                value: FORCE_MAX_READING + 1
            })
        );
    }

    #[test]
    fn encode_unchecked_matches_encode_for_valid_readings() {
        let readings = [101, 202, 303, 404, 505, 606, 707, 808, 909, 1001];
        let expected = ForceDataPacket::new(readings).unwrap().encode().unwrap();
        let mut actual = [0u8; FORCE_DATA_PACKET_LEN];

        encode_readings_unchecked(&readings, &mut actual);

        assert_eq!(actual, expected);
    }
}
