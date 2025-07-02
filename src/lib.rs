#![no_std]

pub mod board;
pub mod config;
pub mod drivers;
pub mod ipc;
pub mod tasks;

pub use board::Board;
pub use drivers::imu::ImuData;
