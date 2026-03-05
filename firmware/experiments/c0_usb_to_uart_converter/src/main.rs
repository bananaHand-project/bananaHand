#![no_std]
#![no_main]

use cobs::{decode, encode};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::usart::{Config as UartConfig, Uart, UartTx};
#[cfg(feature = "defmt")]
use defmt::info;

const UART_BAUDRATE: u32 = 115_200;
const COBS_DELIM: u8 = 0x00;
const MSG_TYPE_POSITION: u8 = 0x01;
const POSITION_COUNT: usize = 8;
const POSITION_LEN: usize = POSITION_COUNT * 2;
const BODY_LEN: usize = 1 + POSITION_LEN + 1; // [type][payload][chk]
const MAX_ENC_FRAME: usize = 64;

fn checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // ST-LINK VCP is typically wired to USART2 (PA3 RX, PA2 TX) on C0 Nucleo boards.
    let mut stlink_cfg = UartConfig::default();
    stlink_cfg.baudrate = UART_BAUDRATE;
    let mut stlink_uart = Uart::new_blocking(p.USART2, p.PA3, p.PA2, stlink_cfg).unwrap();

    // Forward stream to G4 reader on USART1 TX (PC14).
    let mut g4_cfg = UartConfig::default();
    g4_cfg.baudrate = UART_BAUDRATE;
    let mut g4_uart_tx = UartTx::new_blocking(p.USART1, p.PB6, g4_cfg).unwrap();

    let mut byte = [0u8; 1];
    let mut enc_buf = [0u8; MAX_ENC_FRAME];
    let mut enc_len = 0usize;
    let mut dec_buf = [0u8; BODY_LEN];
    let mut out_body = [0u8; BODY_LEN];
    let mut out_enc = [0u8; MAX_ENC_FRAME];

    loop {
        if stlink_uart.blocking_read(&mut byte).is_err() {
            continue;
        }

        let b = byte[0];
        if b != COBS_DELIM {
            if enc_len < MAX_ENC_FRAME {
                enc_buf[enc_len] = b;
                enc_len += 1;
            } else {
                // Drop oversized encoded frame and resync at next delimiter.
                enc_len = 0;
            }
            continue;
        }

        // Delimiter hit, ignore empty frames.
        if enc_len == 0 {
            continue;
        }

        let report = match decode(&enc_buf[..enc_len], &mut dec_buf) {
            Ok(r) => r,
            Err(_) => {
                enc_len = 0;
                continue;
            }
        };
        enc_len = 0;

        let dec_len = report.frame_size();
        if dec_len != BODY_LEN {
            continue;
        }

        let msg_type = dec_buf[0];
        if msg_type != MSG_TYPE_POSITION {
            continue;
        }

        let payload = &dec_buf[1..(1 + POSITION_LEN)];
        let recv_chk = dec_buf[1 + POSITION_LEN];
        if checksum(payload) != recv_chk {
            continue;
        }

        // Rebuild exactly like serial_bridge_node.py:
        // [type][payload][chk(payload)] -> COBS encode -> 0x00 delimiter.
        out_body[0] = MSG_TYPE_POSITION;
        out_body[1..(1 + POSITION_LEN)].copy_from_slice(payload);
        out_body[1 + POSITION_LEN] = checksum(payload);

        let out_len = encode(&out_body, &mut out_enc);
        if out_len + 1 > out_enc.len() {
            continue;
        }
        out_enc[out_len] = COBS_DELIM;

        #[cfg(feature = "defmt")]
        info!("Forwarded COBS PositionUpdate len={}", out_len + 1);

        let _ = g4_uart_tx.blocking_write(&out_enc[..out_len + 1]);
    }
}
