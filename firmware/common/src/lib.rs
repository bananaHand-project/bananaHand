#![no_std]

pub mod force_data_packet;

pub use force_data_packet::{
    FORCE_DATA_PACKET_LEN, FORCE_MAX_READING, FORCE_SENSOR_COUNT, ForceDataPacket,
    ForceDataPacketError,
};
