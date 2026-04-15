#![no_std]

pub mod force_data_packet;
pub const FORCE_SENS_BAUD: u32 = 115_200;

pub use force_data_packet::{
    encode_readings_unchecked, FORCE_DATA_PACKET_LEN, FORCE_MAX_READING, FORCE_SENSOR_COUNT, ForceDataPacket,
    ForceDataPacketError,
};
