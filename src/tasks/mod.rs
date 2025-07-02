use defmt::Format;
use embassy_time::Instant;

pub mod imu;
pub mod servo;

pub mod spi_echo_test;
pub mod spi_echo_dsr;
//pub mod spi_interface;
//pub mod spi_proto;

pub use imu::{imu_stats_task, imu_task};
//pub use servo::servo_control_task;
pub use servo::servo_task;

pub use spi_echo_test::spi_echo_task;
pub use spi_echo_dsr::spi_echo_dsr_task;
//pub use spi_interface::spi_slave_task;
//pub use spi_proto::spi_proto_task;

use crate::drivers::imu::ImuData;

#[derive(Debug, Format, Clone)]
pub enum SystemMessage {
    ImuData { timestamp: Instant, data: ImuData },
    SystemAlert(SystemAlert),
}

#[derive(Debug, Format, Clone)]
pub enum SystemAlert {
    ImuError,
    CommunicationTimeout,
    OverTemperature,
    LowVoltage,
}
