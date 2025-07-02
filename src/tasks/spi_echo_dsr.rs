//! SPI Echo Test with DSR Hardware Flow Control - Fixed buffer handling

use crc16::{State, CCITT_FALSE};
use defmt::info;
use embassy_executor::task;
use embassy_stm32::{gpio::Output, mode::Async, spi::SpiSlave};

const FRAME_SIZE: usize = 260;

#[task]
pub async fn spi_echo_dsr_task(
    mut spi: SpiSlave<'static, Async>,
    mut dsr_pin: Output<'static>,
) {
    info!("SPI Echo DSR Task started - Fixed buffer handling");
    
    let mut rx_buffer = [0u8; FRAME_SIZE];  // For receiving commands
    let mut tx_buffer = [0u8; FRAME_SIZE];  // For sending responses
    
    // Initialize DSR LOW (ready to receive commands)
    dsr_pin.set_low();
    info!("DSR LOW - Ready to receive commands");
    
    loop {
        // Phase 1: Receive command into rx_buffer
        spi.transfer_in_place(&mut rx_buffer).await.unwrap();
        
        // Phase 2: Validate and prepare response in tx_buffer
        if rx_buffer[0] == 0xAA && rx_buffer[1] == 0x55 {
            let payload_len = rx_buffer[2] as usize;
            
            if payload_len <= 252 {
                let crc_pos = 5 + payload_len;
                if crc_pos + 2 <= FRAME_SIZE {
                    let received_crc = u16::from_le_bytes([rx_buffer[crc_pos], rx_buffer[crc_pos + 1]]);
                    let calculated_crc = State::<CCITT_FALSE>::calculate(&rx_buffer[..crc_pos]);
                    
                    if calculated_crc == received_crc {
                        // Valid command - prepare echo response in tx_buffer
                        tx_buffer[..crc_pos].copy_from_slice(&rx_buffer[..crc_pos]);
                        
                        // Increment sequence in response
                        tx_buffer[3] = rx_buffer[3].wrapping_add(1);
                        
                        // Recalculate CRC for the response
                        let new_crc = State::<CCITT_FALSE>::calculate(&tx_buffer[..crc_pos]);
                        let crc_bytes = new_crc.to_le_bytes();
                        tx_buffer[crc_pos] = crc_bytes[0];
                        tx_buffer[crc_pos + 1] = crc_bytes[1];
                        
                        // Zero the rest of tx_buffer
                        for i in (crc_pos + 2)..FRAME_SIZE {
                            tx_buffer[i] = 0;
                        }
                        
                        // Signal response ready
                        dsr_pin.set_high();
                        //info!("DSR HIGH - Valid response ready");
                        
                        // Send response (rx_buffer will get NOOP, tx_buffer goes out)
                        spi.transfer(&mut rx_buffer, &mut tx_buffer).await.unwrap();
                        
                        // Ready for next command
                        dsr_pin.set_low();
                        //info!("DSR LOW - Response sent, ready for next");
                        continue;
                    } else {
                        info!("CRC mismatch: {:04X} vs {:04X}", calculated_crc, received_crc);
                    }
                } else {
                    info!("Frame too short for payload len {}", payload_len);
                }
            } else {
                info!("Invalid payload length: {}", payload_len);
            }
        } else if rx_buffer[0] != 0 || rx_buffer[1] != 0 {
            info!("Invalid header: [{:02X}, {:02X}]", rx_buffer[0], rx_buffer[1]);
        }
        
        // Error response - prepare error in tx_buffer
        tx_buffer[0] = 0xAA;
        tx_buffer[1] = 0x55;
        tx_buffer[2] = 1;
        tx_buffer[3] = rx_buffer[3];  // Echo back received sequence
        tx_buffer[4] = 0xFF;
        tx_buffer[5] = 0xFF;
        let crc = State::<CCITT_FALSE>::calculate(&tx_buffer[..6]);
        let crc_bytes = crc.to_le_bytes();
        tx_buffer[6] = crc_bytes[0];
        tx_buffer[7] = crc_bytes[1];
        
        // Zero rest
        for i in 8..FRAME_SIZE {
            tx_buffer[i] = 0;
        }
        
        dsr_pin.set_high();
        info!("DSR HIGH - Error response ready");
        spi.transfer(&mut rx_buffer, &mut tx_buffer).await.unwrap();
        dsr_pin.set_low();
        info!("DSR LOW - Error sent, ready for next");
    }
}