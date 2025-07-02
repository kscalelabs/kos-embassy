//! SPI Full-Duplex Echo Test with Ping-Pong Buffers
//!
//! This task implements a full-duplex SPI slave that echoes incoming commands back to the master.
//! Uses a ping-pong buffer strategy where two buffers (A & B) alternate between TX and RX roles:
//! - On each transfer: simultaneously sends previous reply while receiving new command
//! - Validates incoming messages with header [0xAA, 0x55] and CRC16 checksum
//! - Echoes valid commands back with recalculated CRC
//! - Buffers swap roles each cycle to maintain continuous full-duplex operation
use crc16::{State, CCITT_FALSE};
use embassy_executor::task;
use embassy_stm32::{mode::Async, spi::SpiSlave};

const FRAME: usize = 260;

#[task]
pub async fn spi_echo_task(mut spi: SpiSlave<'static, Async>) {
    let mut buf_a = [0u8; FRAME];
    let mut buf_b = [0u8; FRAME];
    
    let mut tx = &mut buf_a;
    let mut rx = &mut buf_b;

    loop {
        // 1) full-duplex transfer: shift out *tx*, receive new cmd into *rx*
        spi.transfer_in_place(tx).await.unwrap(); // reply_(n-1) goes out

        // 2) validate header & CRC now contained in *tx* (it was rx last round)
        if tx[0..2] == [0xAA, 0x55] {
            let len = tx[2] as usize;
            if len <= 252 && 5 + len + 2 <= FRAME {
                let crc_rx = u16::from_le_bytes([tx[5 + len], tx[6 + len]]);
                if State::<CCITT_FALSE>::calculate(&tx[..5 + len]) == crc_rx {

                    // 3) build reply in *rx* (the buffer just filled by DMA)
                    rx[..5 + len].copy_from_slice(&tx[..5 + len]); // echo demo
                    let crc = State::<CCITT_FALSE>::calculate(&rx[..5 + len]).to_le_bytes();
                    rx[5 + len] = crc[0];
                    rx[5 + len + 1] = crc[1];
                    for b in &mut rx[5 + len + 2..FRAME] {
                        *b = 0;
                    }
                }
            }
        }
        // 4) swap roles: the buffer that *was* RX becomes next TX
        core::mem::swap(&mut tx, &mut rx);
    }
}