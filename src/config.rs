// Centralize all configuration constants
pub const IMU_SAMPLE_RATE_HZ: u32 = 100;
pub const IMU_SAMPLE_PERIOD_MS: u64 = 1000 / IMU_SAMPLE_RATE_HZ as u64;
pub const UART_BAUDRATE: u32 = 1_000_000;
pub const I2C_FREQUENCY_HZ: u32 = 100_000;

// Channel sizes
pub const IMU_CHANNEL_SIZE: usize = 16;
pub const SYSTEM_CHANNEL_SIZE: usize = 32;
