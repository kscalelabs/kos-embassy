#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::interrupt;
//use embassy_stm32::interrupt::{InterruptExt, Priority};
use {defmt_rtt as _, panic_probe as _};

use kos_embassy::{
    drivers::{imu::create_default_imu, servo::FeetechServo},
    tasks::{
        imu_task, imu_stats_task, servo_task, spi_echo_dsr_task, spi_echo_task,
    },
    Board,
};

/*static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn USART2() {
    EXECUTOR_HIGH.on_interrupt()
}*/

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Starting kos-embassy i/o handler with multi-priority executors");
    let board = Board::init();
    let servo = FeetechServo::new(board.feetech_tx, board.feetech_rx);

    // High-priority executor for servo control (priority level 7)
    // interrupt::USART1.set_priority(Priority::P7);
    //let spawner_high_priority = EXECUTOR_HIGH.start(interrupt::USART2);

    spawner.spawn(servo_task(servo)).unwrap();
    info!("Servo task spawned on main executor");

    match create_default_imu(board.i2c1).await {
        Ok(imu) => {
            spawner.spawn(imu_task(imu)).unwrap();
            spawner.spawn(imu_stats_task()).unwrap();
            info!("IMU task spawned on main executor");
        }
        Err(e) => error!("IMU initialization failed: {:?}", e),
    }

    // Spawn KOS SPI Driver
    //spawner.spawn(spi_proto_task(board.spi1, board.data_ready_spi)).unwrap();

    //SPI Testing Tasks (Only Un-Comment One at a Time)
    //--------------------------------------------------
    //spawner.spawn(spi_echo_dsr_task(board.spi1, board.data_ready_spi)).unwrap();
    spawner.spawn(spi_echo_task(board.spi1)).unwrap();

    info!("SPI task spawned on main executor");
    //--------------------------------------------------

    core::future::pending::<()>().await;
}