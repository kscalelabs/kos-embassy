// SCServo / Feetech STS protocol constants + register list.

#![allow(non_upper_case_globals)]

/* ───── Packet anatomy ──────────────────────────────────────────────── */
pub const HDR_BYTE: u8 = 0xFF; // sync byte
pub const HEADER: [u8; 2] = [HDR_BYTE, HDR_BYTE];

/// Indexes within every transmitted / received frame
pub mod idx {
    pub const ID: usize = 2; // servo ID
    pub const LENGTH: usize = 3; // bytes after this field incl. ERR/INST
    pub const INSTRUCTION: usize = 4; // only in tx packet
    pub const ERROR: usize = 4; // only in status packet
    pub const PARAM_0: usize = 5; // first parameter / data byte
}

/// Absolute packet size limits enforced by the legacy firmware
pub const TXPACKET_MAX_LEN: usize = 250;
pub const RXPACKET_MAX_LEN: usize = 250;

/* ────────────────── Basic IDs ─────────────────── */
pub const BROADCAST_ID: u8 = 0xFE; // 254
pub const MAX_ID: u8 = 0xFC; // 252

/* ────────────────── Instructions ──────────────── */
pub const INST_PING: u8 = 1;
pub const INST_READ: u8 = 2;
pub const INST_WRITE: u8 = 3;
pub const INST_REG_WRITE: u8 = 4;
pub const INST_ACTION: u8 = 5;
pub const INST_SYNC_WRITE: u8 = 0x83;
pub const INST_SYNC_READ: u8 = 0x82;

/* ────────────────── Error bits in status packet ─ */
pub const ERRBIT_VOLTAGE: u8 = 1 << 0;
pub const ERRBIT_ANGLE: u8 = 1 << 1;
pub const ERRBIT_OVERHEAT: u8 = 1 << 2;
pub const ERRBIT_OVERELE: u8 = 1 << 3;
pub const ERRBIT_OVERLOAD: u8 = 1 << 5;

/* ────────────────── Baud‑rate codes ───────────── */
pub const BAUD_1M: u8 = 0;
pub const BAUD_500K: u8 = 1;
pub const BAUD_250K: u8 = 2;
pub const BAUD_128K: u8 = 3;
pub const BAUD_115K2: u8 = 4;
pub const BAUD_76K8: u8 = 5;
pub const BAUD_57K6: u8 = 6;
pub const BAUD_38K4: u8 = 7;

/* ────────────────── Register addresses ────────── */
/* EEPROM */
pub const REG_MODEL: u8 = 3;
pub const REG_ID: u8 = 5;
pub const REG_BAUD_RATE: u8 = 6;
pub const REG_RETURN_DELAY_TIME: u8 = 7;
pub const REG_STATUS_RETURN_LEVEL: u8 = 8;
pub const REG_MIN_ANGLE_LIMIT_L: u8 = 9;
pub const REG_MIN_ANGLE_LIMIT_H: u8 = 10;
pub const REG_MAX_ANGLE_LIMIT_L: u8 = 11;
pub const REG_MAX_ANGLE_LIMIT_H: u8 = 12;
pub const REG_TEMP_LIMIT: u8 = 13;
pub const REG_VOLTAGE_LIMIT: u8 = 14;
pub const REG_MIN_VOLTAGE_LIMIT: u8 = 15;
pub const REG_MAX_TORQUE_LIMIT: u8 = 16;
pub const REG_PHASE: u8 = 18;
pub const REG_UNLOADING_COND: u8 = 19;
pub const REG_LED_ALARM_COND: u8 = 20;
pub const REG_KP: u8 = 21;
pub const REG_KD: u8 = 22;
pub const REG_KI: u8 = 23;
pub const REG_MIN_STARTUP_FORCE: u8 = 24;
/* SRAM */
pub const REG_CW_DEAD: u8 = 26;
pub const REG_CCW_DEAD: u8 = 27;
pub const REG_PROTECTION_CURRENT: u8 = 28;
pub const REG_ANGULAR_RESOLUTION: u8 = 30;
pub const REG_OFS_L: u8 = 31;
pub const REG_OFS_H: u8 = 32;
pub const REG_MODE: u8 = 33;
pub const REG_PROTECTIVE_TORQUE: u8 = 34;
pub const REG_PROTECTION_TIME: u8 = 35;
pub const REG_OVERLOAD_TORQUE: u8 = 36;
pub const REG_SPEED_LOOP_P: u8 = 37;
pub const REG_OVER_CURRENT_TIME: u8 = 38;
pub const REG_SPEED_LOOP_I: u8 = 39;

pub const REG_TORQUE_ENABLE: u8 = 40;
pub const REG_ACC: u8 = 41;
pub const REG_GOAL_POSITION_L: u8 = 42;
pub const REG_GOAL_POSITION_H: u8 = 43;
pub const REG_GOAL_TIME_L: u8 = 44;
pub const REG_GOAL_TIME_H: u8 = 45;
pub const REG_GOAL_SPEED_L: u8 = 46;
pub const REG_GOAL_SPEED_H: u8 = 47;
pub const REG_LOCK: u8 = 55;

pub const REG_PRESENT_POSITION_L: u8 = 56;
pub const REG_PRESENT_POSITION_H: u8 = 57;
pub const REG_PRESENT_SPEED_L: u8 = 58;
pub const REG_PRESENT_SPEED_H: u8 = 59;
pub const REG_PRESENT_LOAD_L: u8 = 60;
pub const REG_PRESENT_LOAD_H: u8 = 61;
pub const REG_PRESENT_VOLTAGE: u8 = 62;
pub const REG_PRESENT_TEMP: u8 = 63;
pub const REG_STATUS: u8 = 65;
pub const REG_MOVING: u8 = 66;
pub const REG_PRESENT_CURRENT_L: u8 = 69;
pub const REG_PRESENT_CURRENT_H: u8 = 70;

/* Factory‑defaults (EEPROM) */
pub const REG_DEF_MOVING_TH: u8 = 80;
pub const REG_DEF_DTS_MS: u8 = 81;
pub const REG_DEF_VK_MS: u8 = 82;
pub const REG_DEF_VMIN: u8 = 83;
pub const REG_DEF_VMAX: u8 = 84;
pub const REG_DEF_AMAX: u8 = 85;
pub const REG_DEF_KACC: u8 = 86;

/* ────────────────── Parameter table ───────────── */
#[derive(Clone, Copy)]
pub enum Signed {
    Unsigned,
    Signed, // only used for int16 regs
}

pub struct Parameter {
    pub name: &'static str,
    pub addr: u8,
    pub size: u8,
    pub signed: Signed,
}

/// Compile‑time list that mirrors the original `servoRegs` (no heap).
pub const SERVO_REGS: &[Parameter] = &[
    Parameter {
        name: "Model",
        addr: REG_MODEL,
        size: 2,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "ID",
        addr: REG_ID,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Baudrate",
        addr: REG_BAUD_RATE,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Return Delay",
        addr: REG_RETURN_DELAY_TIME,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Response Status",
        addr: REG_STATUS_RETURN_LEVEL,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Min Angle Limit",
        addr: REG_MIN_ANGLE_LIMIT_L,
        size: 2,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Max Angle Limit",
        addr: REG_MAX_ANGLE_LIMIT_L,
        size: 2,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Max Temp Limit",
        addr: REG_TEMP_LIMIT,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Max Voltage Limit",
        addr: REG_VOLTAGE_LIMIT,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Min Voltage Limit",
        addr: REG_MIN_VOLTAGE_LIMIT,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Max Torque Limit",
        addr: REG_MAX_TORQUE_LIMIT,
        size: 2,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Phase",
        addr: REG_PHASE,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Unloading Cond",
        addr: REG_UNLOADING_COND,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "LED Alarm Cond",
        addr: REG_LED_ALARM_COND,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "P Coefficient",
        addr: REG_KP,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "D Coefficient",
        addr: REG_KD,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "I Coefficient",
        addr: REG_KI,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Min Startup Force",
        addr: REG_MIN_STARTUP_FORCE,
        size: 2,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "CW Dead Zone",
        addr: REG_CW_DEAD,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "CCW Dead Zone",
        addr: REG_CCW_DEAD,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Protection Current",
        addr: REG_PROTECTION_CURRENT,
        size: 2,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Angular Resolution",
        addr: REG_ANGULAR_RESOLUTION,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Offset",
        addr: REG_OFS_L,
        size: 2,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Mode",
        addr: REG_MODE,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Protective Torque",
        addr: REG_PROTECTIVE_TORQUE,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Protection Time",
        addr: REG_PROTECTION_TIME,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Overload Torque",
        addr: REG_OVERLOAD_TORQUE,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Speed loop P",
        addr: REG_SPEED_LOOP_P,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Over‑Current Time",
        addr: REG_OVER_CURRENT_TIME,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Speed loop I",
        addr: REG_SPEED_LOOP_I,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Torque Enable",
        addr: REG_TORQUE_ENABLE,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Acceleration",
        addr: REG_ACC,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Goal Position",
        addr: REG_GOAL_POSITION_L,
        size: 2,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Goal Time",
        addr: REG_GOAL_TIME_L,
        size: 2,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Goal Speed",
        addr: REG_GOAL_SPEED_L,
        size: 2,
        signed: Signed::Signed,
    },
    Parameter {
        name: "Lock",
        addr: REG_LOCK,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Present Position",
        addr: REG_PRESENT_POSITION_L,
        size: 2,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Present Speed",
        addr: REG_PRESENT_SPEED_L,
        size: 2,
        signed: Signed::Signed,
    },
    Parameter {
        name: "Present Load",
        addr: REG_PRESENT_LOAD_L,
        size: 2,
        signed: Signed::Signed,
    },
    Parameter {
        name: "Present Voltage",
        addr: REG_PRESENT_VOLTAGE,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Present Temp",
        addr: REG_PRESENT_TEMP,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Status",
        addr: REG_STATUS,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Moving",
        addr: REG_MOVING,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Present Current",
        addr: REG_PRESENT_CURRENT_L,
        size: 2,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Def Move Thresh",
        addr: REG_DEF_MOVING_TH,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Def DTS",
        addr: REG_DEF_DTS_MS,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Def VK",
        addr: REG_DEF_VK_MS,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Def VMIN",
        addr: REG_DEF_VMIN,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Def VMAX",
        addr: REG_DEF_VMAX,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Def AMAX",
        addr: REG_DEF_AMAX,
        size: 1,
        signed: Signed::Unsigned,
    },
    Parameter {
        name: "Def KACC",
        addr: REG_DEF_KACC,
        size: 1,
        signed: Signed::Unsigned,
    },
];
