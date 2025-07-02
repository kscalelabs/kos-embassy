use crate::drivers::servo::FeetechServo;
use crate::ipc::ServoTable;
use crate::ipc::SERVO_CH;
use embassy_executor::task;

use crate::drivers::servo::Command as ServoCmd;
use crate::ipc::{update_servo_states, SPI_TO_CONTROL};
use heapless::Vec;

use defmt::{info, warn};
use embassy_time::{with_timeout, Duration, Instant, Timer};

#[task]
pub async fn servo_task(mut servo: FeetechServo) {
    defmt::info!("atarting servo task");
    let mut spi_to_control_seq = 0u32;

     // Scan for available servos at startup
     let mut available_servos = heapless::Vec::<u8, 32>::new();
     servo.scan_into(0, 253, &mut available_servos).await;
 
     if available_servos.is_empty() {
         warn!("No servos found during scan!");
         // Still continue - maybe servos will be connected later
     } else {
         info!(
             "Found {} servos: {:?}",
             available_servos.len(),
             available_servos.as_slice()
         );
     }
    loop {
        match SERVO_CH.receive().await {
            ServoCmd::Scan { reply } => {
                let mut list = Vec::<u8, 32>::new();
                servo.scan_into(0, 253, &mut list).await;
                defmt::info!("SERVO ➜ scan finished, {} id(s)", list.len());
                let _ = reply.send(list).await;
            }
            ServoCmd::Configure {
                id,
                kp,
                kd,
                ki,
                acc,
                torq,
                reply,
            } => {
                let res = servo.configure_actuator(id, kp, kd, ki, acc, torq).await;
                let _ = reply.send(res).await;
                defmt::info!("REQ: configure actuators");
            }
            ServoCmd::ReadAll { id, reply } => {
                let mut tbl: ServoTable = ServoTable::new();
                servo.read_all_servo_params_into(id, &mut tbl).await;
                let _ = reply.send(Ok(tbl)).await;
                defmt::info!("REQ: Servo Param Dumpq");
            }

            // ---------- WriteTargets ------------------------------------
            ServoCmd::WriteTargets { targets: _, reply } => {
                let mut targets_buffer = heapless::Vec::<(u8, u16, u16), 20>::new();

                // Read targets from IPC ring buffer (ignore the command's targets field)
                let has_new_targets = unsafe {
                    SPI_TO_CONTROL
                        .targets
                        .read_targets(&mut targets_buffer, &mut spi_to_control_seq);
                };

                //let res = if has_new_targets && !targets_buffer.is_empty() {
                servo.write_targets(targets_buffer.iter().copied()).await;
                //} else {
                //    Ok(()) // No targets to write
                //};

                //let _ = reply.send(res).await;
            }

            ServoCmd::ReadStates { ids, reply } => {
                // Read states from actual servo hardware
                let servo_states_result = servo.read_states(&ids).await;

                match servo_states_result {
                    Ok(states) => {
                        // Convert to IPC format and write to ring buffer
                        let mut states_buffer = heapless::Vec::<(u8, i16, i16), 20>::new();
                        for &(id, pos, speed) in &states {
                            let _ = states_buffer.push((id, pos, speed));
                        }

                        // Write states to IPC for SPI to read
                        update_servo_states(&states_buffer);

                        info!("Read {} servo states and wrote to IPC", states_buffer.len());
                        let _ = reply.send(Ok(states)).await;
                    }
                    Err(e) => {
                        warn!("Read states failed: {:?}", e);
                        let _ = reply.send(Err(e)).await;
                    }
                }
            }
        }
    }
}

#[task]
pub async fn servo_control_task(mut servo: FeetechServo) {
    info!("Starting servo control loop - scanning for servos...");

    // Scan for available servos at startup
    let mut available_servos = heapless::Vec::<u8, 32>::new();
    servo.scan_into(0, 253, &mut available_servos).await;

    if available_servos.is_empty() {
        warn!("No servos found during scan!");
        // Still continue - maybe servos will be connected later
    } else {
        info!(
            "Found {} servos: {:?}",
            available_servos.len(),
            available_servos.as_slice()
        );
    }

    const LOOP_PERIOD_US: u64 = 10_000; // 100Hz = 10ms
    const MAX_SERVO_OP_TIME: Duration = Duration::from_millis(8);

    let mut next_deadline;
    let mut cycle_count = 0u32;
    let mut max_execution_time_us = 0u64;
    let mut timeout_count = 0u32;
    let mut targets_buffer = heapless::Vec::<(u8, u16, u16), 20>::new();
    let mut states_buffer = heapless::Vec::<(u8, i16, i16), 20>::new();

    let mut spi_to_control_seq = 0u32;

    // Timing metrics
    let mut ipc_read_time_us;
    let mut servo_write_time_us;
    let mut servo_read_time_us;
    let mut ipc_write_time_us = 0u64;

    loop {
        let cycle_start = Instant::now();
        next_deadline = cycle_start + Duration::from_micros(LOOP_PERIOD_US);

        // ═══════════════════════════════════════════════════════════════════
        // CRITICAL SECTION: Servo control work
        // ═══════════════════════════════════════════════════════════════════

        // 1. IPC Read (targets from SPI)
        let ipc_read_start = Instant::now();
        let has_new_targets = unsafe {
            SPI_TO_CONTROL
                .targets
                .read_targets(&mut targets_buffer, &mut spi_to_control_seq)
        };
        ipc_read_time_us = ipc_read_start.elapsed().as_micros();

        // 2. Servo Write (send targets to servos)
        servo_write_time_us = 0;
        if has_new_targets && !targets_buffer.is_empty() {
            let servo_write_start = Instant::now();
            match with_timeout(
                MAX_SERVO_OP_TIME,
                servo.write_targets(targets_buffer.iter().copied()),
            )
            .await
            {
                Ok(Ok(_)) => {
                    servo_write_time_us = servo_write_start.elapsed().as_micros();
                }
                Ok(Err(e)) => {
                    servo_write_time_us = servo_write_start.elapsed().as_micros();
                    warn!("Write targets failed: {:?}", e);
                }
                Err(_) => {
                    servo_write_time_us = MAX_SERVO_OP_TIME.as_micros();
                    timeout_count += 1;
                    warn!("Write targets timeout! (count: {})", timeout_count);
                }
            }
            //Timer::after_micros(2000).await; // Let SPI task run
        }

        // 3. Servo Read (get states from servos)
        servo_read_time_us = 0;
        if !available_servos.is_empty() {
            let servo_read_start = Instant::now();
            let servo_ids: heapless::Vec<u8, 32> = available_servos.clone();

            match with_timeout(MAX_SERVO_OP_TIME, servo.read_states::<32>(&servo_ids)).await {
                Ok(Ok(states)) => {
                    servo_read_time_us = servo_read_start.elapsed().as_micros();

                    // Convert to our buffer format
                    states_buffer.clear();
                    for &(id, pos, speed) in &states {
                        let _ = states_buffer.push((id, pos, speed));
                    }

                    // 4. IPC Write (send states to SPI)
                    let ipc_write_start = Instant::now();
                    update_servo_states(&states_buffer);
                    ipc_write_time_us = ipc_write_start.elapsed().as_micros();
                }
                Ok(Err(e)) => {
                    servo_read_time_us = servo_read_start.elapsed().as_micros();
                    warn!("Read states failed: {:?}", e);
                }
                Err(_) => {
                    servo_read_time_us = MAX_SERVO_OP_TIME.as_micros();
                    timeout_count += 1;
                    warn!(
                        "Read states timeout! (count: {}) for {} servos",
                        timeout_count,
                        available_servos.len()
                    );
                }
            }
        }

        // ═══════════════════════════════════════════════════════════════════
        // END CRITICAL SECTION
        // ═══════════════════════════════════════════════════════════════════

        let execution_time = cycle_start.elapsed();
        let execution_time_us = execution_time.as_micros();

        if execution_time_us > max_execution_time_us {
            max_execution_time_us = execution_time_us;
        }

        cycle_count += 1;

        // Comprehensive timing diagnostics
        if cycle_count % 1000 == 0 {
            info!("Servo control stats (10s): max_exec={}μs, servo_write={}μs, servo_read={}μs, ipc_read={}μs, ipc_write={}μs, targets={}, servos={}, timeouts={}", 
                  max_execution_time_us, servo_write_time_us, servo_read_time_us, ipc_read_time_us, ipc_write_time_us, targets_buffer.len(), available_servos.len(), timeout_count);
            max_execution_time_us = 0;
            timeout_count = 0;
        }

        // Sleep until next deadline
        let now = Instant::now();
        if now < next_deadline {
            Timer::at(next_deadline).await;
        }
    }
}
