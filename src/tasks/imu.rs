use crate::ipc::{ImuSnapshot, IMU_CH, LATEST_IMU_PTR, SYSTEM_CH};
use crate::{
    config::IMU_SAMPLE_PERIOD_MS,
    drivers::imu::{Bno055, ImuData},
};
use defmt::*;
use embassy_executor::task;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker};
use portable_atomic::Ordering;

use super::{SystemAlert, SystemMessage};

struct ImuBuffers {
    buffer_a: ImuSnapshot,
    buffer_b: ImuSnapshot,
    use_a: bool,
}

impl ImuBuffers {
    const fn new() -> Self {
        Self {
            buffer_a: ImuSnapshot {
                ax: 0.0,
                ay: 0.0,
                az: 0.0,
                gx: 0.0,
                gy: 0.0,
                gz: 0.0,
                qw: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                calib_sys: 0,
                calib_gyro: 0,
                calib_accel: 0,
                calib_mag: 0,
            },
            buffer_b: ImuSnapshot {
                ax: 0.0,
                ay: 0.0,
                az: 0.0,
                gx: 0.0,
                gy: 0.0,
                gz: 0.0,
                qw: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                calib_sys: 0,
                calib_gyro: 0,
                calib_accel: 0,
                calib_mag: 0,
            },
            use_a: true,
        }
    }

    fn swap_buffer(&mut self) {
        self.use_a = !self.use_a;
    }

    fn get_active_buffer(&mut self) -> &mut ImuSnapshot {
        if self.use_a {
            &mut self.buffer_a
        } else {
            &mut self.buffer_b
        }
    }
}

static IMU_BUFFERS: Mutex<CriticalSectionRawMutex, ImuBuffers> = Mutex::new(ImuBuffers::new());

#[task]
pub async fn imu_task(mut imu: Bno055) {
    info!(
        "IMU task started - sampling at {}ms intervals",
        IMU_SAMPLE_PERIOD_MS
    );
    let mut ticker = Ticker::every(Duration::from_millis(IMU_SAMPLE_PERIOD_MS));
    let mut error_count = 0u32;
    let mut consecutive_errors = 0u32;

    loop {
        ticker.next().await;

        match imu.read_all_data().await {
            Ok(data) => {
                debug!(
                    "IMU Data:\n\
                     Accel: x={} y={} z={} m/s²\n\
                     Gyro:  x={} y={} z={} rad/s\n\
                     Mag:   x={} y={} z={} μT\n\
                     Quat:  w={} x={} y={} z={}\n\
                     Temp:  {}°C\n\
                     Calib: sys={} gyro={} accel={} mag={}",
                    data.accel.x,
                    data.accel.y,
                    data.accel.z,
                    data.gyro.x,
                    data.gyro.y,
                    data.gyro.z,
                    data.mag.x,
                    data.mag.y,
                    data.mag.z,
                    data.quat.w,
                    data.quat.x,
                    data.quat.y,
                    data.quat.z,
                    data.temp,
                    data.calib.sys,
                    data.calib.gyro,
                    data.calib.accel,
                    data.calib.mag
                );

                let timestamp = Instant::now();

                // Send to channels
                IMU_CH.try_send((timestamp, data)).ok();
                SYSTEM_CH
                    .try_send(SystemMessage::ImuData { timestamp, data })
                    .ok();

                // Send to IMU channel for stats
                //if IMU_CH.try_send((timestamp, data)).is_err() {
                //    warn!("IMU channel full, dropping sample");
                //}

                // Reset error counters on success
                if consecutive_errors > 0 {
                    info!(
                        "IMU recovered after {} consecutive errors",
                        consecutive_errors
                    );
                    consecutive_errors = 0;
                }

                // Update the active buffer with mutex protection
                let mut buffers = IMU_BUFFERS.lock().await;
                let active_buffer = buffers.get_active_buffer();
                *active_buffer = ImuSnapshot {
                    ax: data.accel.x,
                    ay: data.accel.y,
                    az: data.accel.z,
                    gx: data.gyro.x,
                    gy: data.gyro.y,
                    gz: data.gyro.z,
                    qw: data.quat.w,
                    qx: data.quat.x,
                    qy: data.quat.y,
                    qz: data.quat.z,
                    calib_sys: data.calib.sys,
                    calib_gyro: data.calib.gyro,
                    calib_accel: data.calib.accel,
                    calib_mag: data.calib.mag,
                };

                // Update the pointer atomically
                LATEST_IMU_PTR.store(active_buffer as *mut _, Ordering::Release);

                // Swap buffers for next iteration
                buffers.swap_buffer();
            }
            Err(e) => {
                error_count += 1;
                consecutive_errors += 1;

                if consecutive_errors % 100 == 1 {
                    warn!("IMU read error #{}: {:?}", error_count, e);
                }

                // Alert system if too many consecutive errors
                if consecutive_errors >= 50 {
                    SYSTEM_CH
                        .try_send(SystemMessage::SystemAlert(SystemAlert::ImuError))
                        .ok();
                }
            }
        }
    }
}

#[task]
pub async fn imu_stats_task() {
    info!("IMU stats task started");
    let mut last_sec = Instant::now();
    let mut samples = 0u32;
    let mut duplicates = 0u32;
    let mut prev: Option<ImuData> = None;

    loop {
        let (timestamp, data) = IMU_CH.receive().await;
        samples += 1;

        // Check for duplicate data
        if let Some(ref previous_data) = prev {
            if data.is_duplicate_of(previous_data) {
                duplicates += 1;
            }
        }
        prev = Some(data);

        // Report stats every second
        if timestamp.duration_since(last_sec) >= Duration::from_secs(1) {
            let duplicate_rate = (duplicates * 100) / samples.max(1);
            info!("IMU: {} Hz, {}% duplicates", samples, duplicate_rate);
            samples = 0;
            duplicates = 0;
            last_sec = timestamp;
        }
    }
}
