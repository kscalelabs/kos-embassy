use defmt::*;
use embassy_stm32::i2c;
use embassy_time::{Duration, Timer};

// BNO055 I2C addresses
pub const BNO055_ADDRESS_A: u8 = 0x28;
pub const BNO055_ADDRESS_B: u8 = 0x29;

const BNO055_OPR_MODE_ADDR: u8 = 0x3D;
const BNO055_SYS_TRIGGER_ADDR: u8 = 0x3F;
const BNO055_UNIT_SEL_ADDR: u8 = 0x3B;
const BNO055_SYS_STAT_ADDR: u8 = 0x39;
const BNO055_SYS_ERR_ADDR: u8 = 0x3A;

// BNO055 Register Addresses
const BNO055_CHIP_ID_ADDR: u8 = 0x00;
const BNO055_ACCEL_DATA_X_LSB_ADDR: u8 = 0x08;
const BNO055_MAG_DATA_X_LSB_ADDR: u8 = 0x0E;
const BNO055_GYRO_DATA_X_LSB_ADDR: u8 = 0x14;
const BNO055_QUATERNION_DATA_W_LSB_ADDR: u8 = 0x20;
const BNO055_TEMP_ADDR: u8 = 0x34;
const BNO055_CALIB_STAT_ADDR: u8 = 0x35;

// BNO055 ID
const BNO055_ID: u8 = 0xA0;

// System status values
const SYS_STAT_IDLE: u8 = 0;
const SYS_STAT_SYS_ERROR: u8 = 1;
const SYS_STAT_INIT_PERIPHERALS: u8 = 2;
const SYS_STAT_SYS_INIT: u8 = 3;
const SYS_STAT_EXECUTING_SELFTEST: u8 = 4;
const SYS_STAT_SENSOR_FUSION_RUNNING: u8 = 5;
const SYS_STAT_RUNNING_NO_FUSION: u8 = 6;

// Data structures (unchanged)
#[derive(Debug, Format, Clone, Copy)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Format, Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Format, Clone, Copy)]
pub struct CalibrationStatus {
    pub sys: u8,   // System calibration (0-3)
    pub gyro: u8,  // Gyroscope calibration (0-3)
    pub accel: u8, // Accelerometer calibration (0-3)
    pub mag: u8,   // Magnetometer calibration (0-3)
}

#[derive(Debug, Format, Clone, Copy)]
pub struct ImuData {
    pub accel: Vector3,
    pub gyro: Vector3,
    pub mag: Vector3,
    pub quat: Quaternion,
    pub temp: i8,
    pub calib: CalibrationStatus,
}

impl CalibrationStatus {
    pub fn from_byte(calib_byte: u8) -> Self {
        Self {
            sys: (calib_byte >> 6) & 0x03,
            gyro: (calib_byte >> 4) & 0x03,
            accel: (calib_byte >> 2) & 0x03,
            mag: calib_byte & 0x03,
        }
    }

    pub fn is_fully_calibrated(&self) -> bool {
        self.sys == 3 && self.gyro == 3 && self.accel == 3 && self.mag == 3
    }
}

impl ImuData {
    pub fn is_duplicate_of(&self, other: &Self) -> bool {
        const EPS: f32 = 1e-5;
        (self.accel.x - other.accel.x).abs() < EPS
            && (self.accel.y - other.accel.y).abs() < EPS
            && (self.accel.z - other.accel.z).abs() < EPS
            && (self.gyro.x - other.gyro.x).abs() < EPS
            && (self.gyro.y - other.gyro.y).abs() < EPS
            && (self.gyro.z - other.gyro.z).abs() < EPS
            && (self.mag.x - other.mag.x).abs() < EPS
            && (self.mag.y - other.mag.y).abs() < EPS
            && (self.mag.z - other.mag.z).abs() < EPS
            && (self.quat.w - other.quat.w).abs() < EPS
            && (self.quat.x - other.quat.x).abs() < EPS
            && (self.quat.y - other.quat.y).abs() < EPS
            && (self.quat.z - other.quat.z).abs() < EPS
            && self.temp == other.temp
            && self.calib.sys == other.calib.sys
            && self.calib.gyro == other.calib.gyro
            && self.calib.accel == other.calib.accel
            && self.calib.mag == other.calib.mag
    }
}

pub struct Bno055 {
    i2c: embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async>,
    addr: u8,
}

impl Bno055 {
    pub async fn new(
        i2c: embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async>,
        addr: u8,
    ) -> Result<Self, i2c::Error> {
        let mut bno = Self { i2c, addr };

        info!("Starting BNO055 initialization sequence...");

        // Give the chip time to power up
        Timer::after(Duration::from_millis(100)).await;

        // Initialization with retries (this can likely be simplified)
        for attempt in 1..=5 {
            info!("BNO055 init attempt {}/5", attempt);

            match bno.try_init().await {
                Ok(()) => {
                    info!("BNO055 successfully initialized on attempt {}", attempt);
                    return Ok(bno);
                }
                Err(e) => {
                    warn!("BNO055 init attempt {} failed: {:?}", attempt, e);

                    // Progressive backoff: 200ms, 400ms, 800ms, 1600ms
                    let delay_ms = 200u64 << (attempt - 1);
                    info!("Waiting {}ms before next attempt...", delay_ms);
                    Timer::after(Duration::from_millis(delay_ms)).await;
                }
            }
        }

        error!("BNO055 initialization failed after 5 attempts");
        Err(i2c::Error::Nack)
    }

    async fn try_init(&mut self) -> Result<(), i2c::Error> {
        // Step 1: Try to recover I2C bus state by attempting a read
        info!("Attempting I2C bus recovery...");
        let _ = self.read_byte_with_retries(BNO055_CHIP_ID_ADDR, 3).await;

        // Step 2: Check chip ID with retries
        info!("Checking chip ID...");
        let id = self.read_byte_with_retries(BNO055_CHIP_ID_ADDR, 5).await?;
        if id != BNO055_ID {
            error!(
                "Invalid BNO055 ID: 0x{:02X}, expected 0x{:02X}",
                id, BNO055_ID
            );
            return Err(i2c::Error::Nack);
        }
        info!("BNO055 chip ID verified: 0x{:02X}", id);

        // Step 3: Software reset
        info!("Performing software reset...");
        self.write_byte_with_retries(BNO055_SYS_TRIGGER_ADDR, 0x20, 3)
            .await?;

        // Wait for reset to complete (BNO055 datasheet: up to 650ms)
        Timer::after(Duration::from_millis(800)).await;

        // Step 4: Wait for the chip to be ready
        info!("Waiting for chip to be ready...");
        self.wait_for_chip_ready().await?;

        // Step 5: Set to config mode
        info!("Setting config mode...");
        self.write_byte_with_retries(BNO055_OPR_MODE_ADDR, 0x00, 3)
            .await?;
        Timer::after(Duration::from_millis(30)).await;

        // Step 6: Configure units (orientation, m/s², Dps, °C)
        info!("Configuring units...");
        self.write_byte_with_retries(BNO055_UNIT_SEL_ADDR, 0x80, 3)
            .await?;
        Timer::after(Duration::from_millis(10)).await;

        // Step 7: Set to NDOF mode (all sensors, fusion enabled)
        info!("Setting NDOF mode...");
        self.write_byte_with_retries(BNO055_OPR_MODE_ADDR, 0x0C, 3)
            .await?;
        Timer::after(Duration::from_millis(50)).await;

        // Step 8: Verify we're in the correct mode
        let mode = self.read_byte_with_retries(BNO055_OPR_MODE_ADDR, 3).await?;
        if mode != 0x0C {
            error!("Failed to set NDOF mode: got 0x{:02X}, expected 0x0C", mode);
            return Err(i2c::Error::Nack);
        }

        // Step 9: Final system status check
        self.check_system_status().await?;

        info!("BNO055 initialization completed successfully");
        Ok(())
    }

    async fn wait_for_chip_ready(&mut self) -> Result<(), i2c::Error> {
        // Wait up to 2 seconds for chip to become ready
        for _ in 0..20 {
            if let Ok(id) = self.read_byte(BNO055_CHIP_ID_ADDR).await {
                if id == BNO055_ID {
                    info!("Chip is ready");
                    return Ok(());
                }
            }
            Timer::after(Duration::from_millis(100)).await;
        }
        error!("Timeout waiting for chip to be ready");
        Err(i2c::Error::Timeout)
    }

    async fn check_system_status(&mut self) -> Result<(), i2c::Error> {
        let sys_stat = self.read_byte_with_retries(BNO055_SYS_STAT_ADDR, 3).await?;
        let sys_err = self.read_byte_with_retries(BNO055_SYS_ERR_ADDR, 3).await?;

        info!(
            "System status: 0x{:02X}, System error: 0x{:02X}",
            sys_stat, sys_err
        );

        match sys_stat {
            SYS_STAT_IDLE => info!("System idle"),
            SYS_STAT_SYS_ERROR => {
                error!("System error detected, error code: 0x{:02X}", sys_err);
                return Err(i2c::Error::Nack);
            }
            SYS_STAT_INIT_PERIPHERALS => info!("Initializing peripherals..."),
            SYS_STAT_SYS_INIT => info!("System initialization..."),
            SYS_STAT_EXECUTING_SELFTEST => info!("Executing self-test..."),
            SYS_STAT_SENSOR_FUSION_RUNNING => info!("Sensor fusion running"),
            SYS_STAT_RUNNING_NO_FUSION => info!("Running without fusion"),
            _ => warn!("Unknown system status: 0x{:02X}", sys_stat),
        }
        Ok(())
    }

    async fn read_byte_with_retries(&mut self, reg: u8, retries: u8) -> Result<u8, i2c::Error> {
        let mut last_error = i2c::Error::Timeout;

        for attempt in 1..=retries {
            match self.read_byte(reg).await {
                Ok(value) => return Ok(value),
                Err(e) => {
                    last_error = e;
                    if attempt < retries {
                        Timer::after(Duration::from_millis(10)).await;
                    }
                }
            }
        }

        Err(last_error)
    }

    async fn write_byte_with_retries(
        &mut self,
        reg: u8,
        value: u8,
        retries: u8,
    ) -> Result<(), i2c::Error> {
        let mut last_error = i2c::Error::Timeout;

        for attempt in 1..=retries {
            match self.write_byte(reg, value).await {
                Ok(()) => return Ok(()),
                Err(e) => {
                    last_error = e;
                    if attempt < retries {
                        Timer::after(Duration::from_millis(10)).await;
                    }
                }
            }
        }

        Err(last_error)
    }

    pub async fn read_all_data(&mut self) -> Result<ImuData, i2c::Error> {
        // Read accelerometer (6 bytes)
        let accel_raw = self.read_bytes::<6>(BNO055_ACCEL_DATA_X_LSB_ADDR).await?;
        let accel = Vector3 {
            x: i16::from_le_bytes([accel_raw[0], accel_raw[1]]) as f32 / 100.0,
            y: i16::from_le_bytes([accel_raw[2], accel_raw[3]]) as f32 / 100.0,
            z: i16::from_le_bytes([accel_raw[4], accel_raw[5]]) as f32 / 100.0,
        };

        // Read magnetometer (6 bytes)
        let mag_raw = self.read_bytes::<6>(BNO055_MAG_DATA_X_LSB_ADDR).await?;
        let mag = Vector3 {
            x: i16::from_le_bytes([mag_raw[0], mag_raw[1]]) as f32 / 16.0,
            y: i16::from_le_bytes([mag_raw[2], mag_raw[3]]) as f32 / 16.0,
            z: i16::from_le_bytes([mag_raw[4], mag_raw[5]]) as f32 / 16.0,
        };

        // Read gyroscope (6 bytes)
        let gyro_raw = self.read_bytes::<6>(BNO055_GYRO_DATA_X_LSB_ADDR).await?;
        let gyro = Vector3 {
            x: i16::from_le_bytes([gyro_raw[0], gyro_raw[1]]) as f32 / 16.0,
            y: i16::from_le_bytes([gyro_raw[2], gyro_raw[3]]) as f32 / 16.0,
            z: i16::from_le_bytes([gyro_raw[4], gyro_raw[5]]) as f32 / 16.0,
        };

        // Read quaternion (8 bytes)
        let quat_raw = self
            .read_bytes::<8>(BNO055_QUATERNION_DATA_W_LSB_ADDR)
            .await?;
        let quat_scale = 1.0 / (1 << 14) as f32;
        let quat = Quaternion {
            w: i16::from_le_bytes([quat_raw[0], quat_raw[1]]) as f32 * quat_scale,
            x: i16::from_le_bytes([quat_raw[2], quat_raw[3]]) as f32 * quat_scale,
            y: i16::from_le_bytes([quat_raw[4], quat_raw[5]]) as f32 * quat_scale,
            z: i16::from_le_bytes([quat_raw[6], quat_raw[7]]) as f32 * quat_scale,
        };

        // Read temperature
        let temp = self.read_byte(BNO055_TEMP_ADDR).await? as i8;

        let calib_byte = self.read_byte(BNO055_CALIB_STAT_ADDR).await?;
        let calib = CalibrationStatus::from_byte(calib_byte);

        Ok(ImuData {
            accel,
            gyro,
            mag,
            quat,
            temp,
            calib,
        })
    }

    async fn write_byte(&mut self, reg: u8, value: u8) -> Result<(), i2c::Error> {
        self.i2c.write(self.addr, &[reg, value]).await
    }

    async fn read_byte(&mut self, reg: u8) -> Result<u8, i2c::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    async fn read_bytes<const N: usize>(&mut self, reg: u8) -> Result<[u8; N], i2c::Error> {
        let mut buf = [0u8; N];
        self.i2c.write_read(self.addr, &[reg], &mut buf).await?;
        Ok(buf)
    }
}

// Factory function to create the default IMU (BNO055)
pub async fn create_default_imu(
    i2c: embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async>,
) -> Result<Bno055, i2c::Error> {
    Bno055::new(i2c, BNO055_ADDRESS_A).await
}
