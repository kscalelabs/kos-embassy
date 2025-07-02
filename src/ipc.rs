#![allow(dead_code)]
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as RawMutex, channel::Channel};

use crate::config::*;
use bytemuck::{Pod, Zeroable};
use embassy_time::Instant;

use crate::drivers::imu::ImuData;
use crate::drivers::servo::{Command as ServoCmd, TxRxError, SERVO_TABLE_MAX};
use crate::tasks::SystemMessage;
use heapless::Vec;
use portable_atomic::{fence, AtomicPtr, AtomicU32, Ordering};

#[repr(C)]
#[derive(Clone, Copy)]
pub struct ServoStatesSnapshot {
    pub count: u8,
    pub states: [(u8, i16, i16); 32], // (id, position, speed)
}

pub static LATEST_SERVO_STATES_PTR: AtomicPtr<ServoStatesSnapshot> =
    AtomicPtr::new(core::ptr::null_mut());
static mut SERVO_STATES_A: ServoStatesSnapshot = ServoStatesSnapshot {
    count: 0,
    states: [(0, 0, 0); 32],
};
static mut SERVO_STATES_B: ServoStatesSnapshot = ServoStatesSnapshot {
    count: 0,
    states: [(0, 0, 0); 32],
};

pub fn update_servo_states(new_states: &[(u8, i16, i16)]) {
    let current_ptr = LATEST_SERVO_STATES_PTR.load(Ordering::Acquire);
    let next_buf = unsafe {
        if current_ptr == &SERVO_STATES_A as *const _ as *mut _ {
            &mut SERVO_STATES_B
        } else {
            &mut SERVO_STATES_A
        }
    };

    // Update the inactive buffer
    next_buf.count = new_states.len() as u8;
    next_buf.states[..new_states.len()].copy_from_slice(new_states);

    // Atomic switch
    LATEST_SERVO_STATES_PTR.store(next_buf as *mut _, Ordering::Release);
}

// Simple, fast data structures - no atomics inside
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ServoTarget {
    pub id: u8,
    pub position: u16,
    pub speed: u16,
}

// Single-writer (SPI) to single-reader (Control) for targets
#[repr(align(64))] // Cache line aligned
pub struct ServoTargetRing {
    // Only ONE atomic for synchronization
    sequence: AtomicU32,
    count: u8,
    _pad: [u8; 59], // Pad to separate cache lines
    targets: [ServoTarget; 20],
}

impl ServoTargetRing {
    pub const fn new() -> Self {
        Self {
            sequence: AtomicU32::new(0),
            count: 0,
            _pad: [0; 59],
            targets: [ServoTarget {
                id: 0,
                position: 0,
                speed: 0,
            }; 20],
        }
    }

    // SPI task writes new targets
    pub fn write_targets(&mut self, targets: &[(u8, u16, u16)]) {
        // 1. Write plain data (no atomics!)
        self.count = targets.len().min(20) as u8;
        for (i, &(id, pos, speed)) in targets.iter().enumerate().take(20) {
            self.targets[i] = ServoTarget {
                id,
                position: pos,
                speed,
            };
        }

        // 2. Memory barrier to ensure writes complete
        fence(Ordering::Release);

        // 3. Single atomic increment to signal new data
        self.sequence.fetch_add(1, Ordering::Release);
    }

    // Control task reads targets if new data available
    pub fn read_targets(
        &self,
        out: &mut heapless::Vec<(u8, u16, u16), 20>,
        last_seq: &mut u32,
    ) -> bool {
        let current_seq = self.sequence.load(Ordering::Acquire);

        if current_seq == *last_seq {
            return false; // No new data
        }

        // Memory barrier to ensure we see latest data
        fence(Ordering::Acquire);

        // Fast bulk copy - no atomics!
        out.clear();
        for i in 0..self.count as usize {
            let target = self.targets[i];
            let _ = out.push((target.id, target.position, target.speed));
        }

        *last_seq = current_seq;
        true // New data was read
    }
}

// Updated ring structures
pub struct SpiToControlRing {
    pub targets: ServoTargetRing,
}

impl SpiToControlRing {
    pub const fn new() -> Self {
        Self {
            targets: ServoTargetRing::new(),
        }
    }
}

pub static mut SPI_TO_CONTROL: SpiToControlRing = SpiToControlRing::new();

pub type ServoTable = heapless::Vec<u8, { SERVO_TABLE_MAX }>;

pub static SERVO_TABLE_REPLY: Channel<RawMutex, Result<ServoTable, TxRxError>, 1> = Channel::new();

/* Servo command channel */
pub static SERVO_CH: Channel<RawMutex, ServoCmd, 4> = Channel::new();
pub static SERVO_REPLY_CH: Channel<RawMutex, Result<(), TxRxError>, 1> = Channel::new();
pub static SERVO_SCAN_REPLY_CH: Channel<
    RawMutex,
    Vec<u8, 32>, // ← tuple
    1,
> = Channel::new();

pub const SERVO_STATE_MAX: usize = 32;
pub static SERVO_STATE_REPLY_CH: Channel<
    RawMutex,
    Result<heapless::Vec<(u8, i16, i16), SERVO_STATE_MAX>, TxRxError>,
    1,
> = Channel::new();
/*  telemetry channels */
pub static IMU_CH: Channel<RawMutex, (Instant, ImuData), IMU_CHANNEL_SIZE> = Channel::new();
pub static SYSTEM_CH: Channel<RawMutex, SystemMessage, SYSTEM_CHANNEL_SIZE> = Channel::new();

/// Wire-format packet: exactly what the SPI task will copy out over the bus.
#[repr(C)]
#[derive(Clone, Copy, Default, Zeroable, Pod)]
pub struct ImuSnapshot {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    pub qw: f32,
    pub qx: f32,
    pub qy: f32,
    pub qz: f32,
    pub calib_sys: u8,
    pub calib_gyro: u8,
    pub calib_accel: u8,
    pub calib_mag: u8,
}

pub static LATEST_IMU_PTR: AtomicPtr<ImuSnapshot> = AtomicPtr::new(core::ptr::null_mut());
