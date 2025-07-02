//! Feetech STS‑series servo driver
//! ===========================================================

use super::protocol as p;
use crate::ipc::SERVO_STATE_MAX;
use embassy_stm32::{
    mode::Async,
    usart::{Error as UartError, RingBufferedUartRx, UartTx},
};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_io_async::{Read, Write};
use heapless::Vec;

static mut WRITE_BUFFER: [u8; 256] = [0; 256];
type OneShotSender<T> = embassy_sync::channel::Sender<
    'static,
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    T,
    1,
>;

pub enum Command {
    Scan {
        reply: OneShotSender<Vec<u8, 32>>,
    },
    Configure {
        id: u8,
        kp: u8,
        kd: u8,
        ki: u8,
        acc: f32,
        torq: bool,
        reply: OneShotSender<Result<(), TxRxError>>,
    },
    ReadAll {
        id: u8,
        reply: OneShotSender<Result<Vec<u8, { SERVO_STATE_MAX }>, TxRxError>>,
    },
    WriteTargets {
        targets: heapless::Vec<(u8, u16, u16), 30>,
        reply: OneShotSender<Result<(), TxRxError>>,
    },
    ReadStates {
        ids: heapless::Vec<u8, { SERVO_STATE_MAX }>,
        reply: OneShotSender<Result<heapless::Vec<(u8, i16, i16), { SERVO_STATE_MAX }>, TxRxError>>,
    },
}
/* ------------------------------------------------------------------------- */
/*  Error enum                                                               */
/* ------------------------------------------------------------------------- */
#[derive(Debug, defmt::Format)]
pub enum TxRxError {
    Uart(UartError),
    Timeout,
    BadChecksum,
    BadFormat,
    BufferFull,
}
impl From<UartError> for TxRxError {
    fn from(e: UartError) -> Self {
        Self::Uart(e)
    }
}

/* ------------------------------------------------------------------------- */
/*  Constants                                                                */
/* ------------------------------------------------------------------------- */
const WRITE_RETRIES: usize = 3;
pub const SERVO_TABLE_MAX: usize = 32; // sync‑read vec capacity

/* ------------------------------------------------------------------------- */
/*  Helper fns                                                               */
/* ------------------------------------------------------------------------- */
#[inline]
fn lo_byte(w: u16) -> u8 {
    w as u8
}
#[inline]
fn hi_byte(w: u16) -> u8 {
    (w >> 8) as u8
}

/* ------------------------------------------------------------------------- */
/*  Main driver struct                                                       */
/* ------------------------------------------------------------------------- */
pub struct FeetechServo {
    tx: UartTx<'static, Async>,
    rx: RingBufferedUartRx<'static>,
}

impl FeetechServo {
    pub fn new(tx: UartTx<'static, Async>, rx: RingBufferedUartRx<'static>) -> Self {
        Self { tx, rx }
    }

    /* ================= low‑level ring helpers ========================= */

    async fn clear_rx_buffer(&mut self) {
        let mut trash = [0u8; 128];
        let start = Instant::now();
        const MAX_CLEAR_TIME: Duration = Duration::from_millis(5); // Limit clearing time

        while start.elapsed() < MAX_CLEAR_TIME {
            match with_timeout(Duration::from_micros(500), self.rx.read(&mut trash)).await {
                Ok(_) => continue, // Keep clearing
                Err(_) => break,   // Timeout - buffer is clear
            }
        }
    }

    async fn read_exact_deadline(&mut self, buf: &mut [u8], dl: Duration) -> Result<(), TxRxError> {
        match with_timeout(dl, self.rx.read_exact(buf)).await {
            Ok(Ok(())) => Ok(()),
            Ok(Err(_)) => Err(TxRxError::Uart(UartError::Overrun)), // Just use Overrun directly
            Err(_) => Err(TxRxError::Timeout),
        }
    }

    async fn write_then_wait(&mut self, pkt: &[u8], settle_us: u64) -> Result<(), TxRxError> {
        self.tx.write_all(pkt).await.map_err(TxRxError::Uart)?;
        if settle_us > 0 {
            Timer::after_micros(settle_us).await;
        }
        Ok(())
    }

    /* ================= public API ============================ */

    /* ---- PING ---- */
    pub async fn ping(&mut self, id: u8) -> Result<(), TxRxError> {
        const LEN: u8 = 2;
        let chk = !(id.wrapping_add(LEN).wrapping_add(p::INST_PING));
        let pkt = [0xFF, 0xFF, id, LEN, p::INST_PING, chk];
        self.clear_rx_buffer().await;
        self.write_then_wait(&pkt, 80).await?;
        //info!("Servo Packet Sent: {:02X}", pkt);

        let mut resp = [0u8; 6];
        self.read_exact_deadline(&mut resp, Duration::from_millis(5))
            .await?;
        if resp[..2] != [0xFF, 0xFF] || resp[2] != id || resp[3] != 2 {
            return Err(TxRxError::BadFormat);
        }
        let sum_ok = {
            let s = resp[2..5].iter().fold(0u8, |a, &b| a.wrapping_add(b));
            !s == resp[5]
        };
        if !sum_ok {
            return Err(TxRxError::BadChecksum);
        }
        //info!("Servo Packet Recv: {:02X}", resp);
        Ok(())
    }

    /* ---- scan_into ---- */
    pub async fn scan_into(&mut self, first: u8, last: u8, out: &mut Vec<u8, 32>) {
        out.clear();
        for id in first..=last {
            if self.ping(id).await.is_ok() {
                out.push(id).ok();
            }
        }
    }

    /* ---- configure_actuator ---- */
    pub async fn configure_actuator(
        &mut self,
        id: u8,
        kp: u8,
        kd: u8,
        ki: u8,
        acc_deg: f32,
        torque: bool,
    ) -> Result<(), TxRxError> {
        let acc = Self::degrees_to_counts(acc_deg, 0.0);
        let acc_b = [lo_byte(acc), hi_byte(acc)];
        async fn wv(s: &mut FeetechServo, id: u8, addr: u8, data: &[u8]) -> Result<(), TxRxError> {
            for _ in 0..WRITE_RETRIES {
                s.write_register(id, addr, data).await?;
                let got = s.read_register::<2>(id, addr, data.len() as u8).await?;
                if got[..data.len()] == *data {
                    return Ok(());
                }
            }
            Err(TxRxError::BadFormat)
        }
        wv(self, id, p::REG_KP, &[kp]).await?;
        wv(self, id, p::REG_KD, &[kd]).await?;
        wv(self, id, 23, &[ki]).await?;
        wv(self, id, p::REG_ACC, &acc_b).await?;
        wv(self, id, p::REG_TORQUE_ENABLE, &[torque as u8]).await?;
        embassy_time::Timer::after_millis(2).await;
        Ok(())
    }

    /* ---- read_all_servo_params_into ---- */
    pub async fn read_all_servo_params_into<const N: usize>(
        &mut self,
        id: u8,
        out: &mut Vec<u8, N>,
    ) {
        use super::protocol::SERVO_REGS;
        out.clear();
        for r in SERVO_REGS {
            if let Ok(bytes) = self.read_register::<2>(id, r.addr, r.size).await {
                out.push(r.addr).ok();
                out.push(r.size).ok();
                out.extend_from_slice(&bytes[..r.size as usize]).ok();
            }
        }
    }

    /* ---- read_states (sync‑read) ---- */
    pub async fn read_states<const MAX: usize>(
        &mut self,
        ids: &[u8],
    ) -> Result<heapless::Vec<(u8, i16, i16), MAX>, TxRxError> {
        if ids.is_empty() {
            return Ok(heapless::Vec::new());
        }

        use p::{HEADER, INST_SYNC_READ};
        const LEN_PER: u8 = 6;

        // Build request packet
        let mut req = heapless::Vec::<u8, 64>::new();
        req.extend_from_slice(&HEADER).unwrap();
        req.extend_from_slice(&[
            p::BROADCAST_ID,
            4 + ids.len() as u8,
            INST_SYNC_READ,
            p::REG_PRESENT_POSITION_L,
            LEN_PER,
        ])
        .unwrap();
        req.extend_from_slice(ids).unwrap();
        let chk = !(req[2..].iter().fold(0u8, |a, &b| a.wrapping_add(b)));
        req.push(chk).unwrap();

        // Clear buffer and send request
        self.clear_rx_buffer().await;
        self.write_then_wait(&req, 200).await?;

        let mut out = heapless::Vec::<(u8, i16, i16), MAX>::new();
        let deadline = Instant::now() + Duration::from_millis(4);

        while out.len() < ids.len() && Instant::now() < deadline {
            let mut pkt = [0u8; 12];

            // Wait for next servo response
            if self
                .read_exact_deadline(&mut pkt, Duration::from_millis(1))
                .await
                .is_err()
            {
                continue;
            }

            // Validate
            if pkt[..2] != p::HEADER || pkt[3] != LEN_PER + 2 {
                continue;
            }

            let servo_id = pkt[2];
            if !ids.contains(&servo_id) || out.iter().any(|(id, _, _)| *id == servo_id) {
                continue;
            }

            // Validate checksum
            let sum = !(pkt[2..11].iter().fold(0u8, |a, &b| a.wrapping_add(b)));
            if sum != pkt[11] {
                continue;
            }

            let pos = i16::from_le_bytes([pkt[5], pkt[6]]);
            let spd = i16::from_le_bytes([pkt[9], pkt[10]]);
            let _ = out.push((servo_id, pos, spd));
        }

        Ok(out)
    }

    /* ---- write_targets (sync‑write) ---- */
    pub async fn write_targets<I>(&mut self, targets: I) -> Result<(), TxRxError>
    where
        I: IntoIterator<Item = (u8, u16, u16)>,
    {
        unsafe {
            let buf = &mut WRITE_BUFFER;
           

            // Header
            buf[0..2].copy_from_slice(&p::HEADER);
            buf[2] = p::BROADCAST_ID;
            buf[4] = p::INST_SYNC_WRITE;
            buf[5] = p::REG_GOAL_POSITION_L;
            buf[6] = 6;
            let mut len = 7;

            // Targets
            for (id, pos, spd) in targets {
                if id > p::MAX_ID || len + 7 > 256 {
                    continue;
                }
                buf[len] = id;
                buf[len + 1] = lo_byte(pos);
                buf[len + 2] = hi_byte(pos);
                buf[len + 3] = 0; // time low  (unused)
                buf[len + 4] = 0; // time high (unused)
                buf[len + 5] = lo_byte(spd);
                buf[len + 6] = hi_byte(spd);
                len += 7;
            }

            // Length & checksum
            buf[3] = (len - 3) as u8;
            let mut chk = 0u8;
            for i in 2..len {
                chk = chk.wrapping_add(buf[i]);
            }
            buf[len] = !chk;
            len += 1;

            // Send
            self.tx
                .write_all(&buf[..len])
                .await
                .map_err(TxRxError::Uart)
        }
    }

    /* ================= register helpers ============================ */
    /** Read `len` bytes from a register and return them in a `heapless::Vec`. */
    async fn read_register<const N: usize>(
        &mut self,
        id: u8,
        addr: u8,
        len: u8,
    ) -> Result<Vec<u8, N>, TxRxError> {
        const TX_LEN: u8 = 4;
        let mut req = [0u8; 8];
        req[..2].copy_from_slice(&p::HEADER);
        req[p::idx::ID] = id;
        req[p::idx::LENGTH] = TX_LEN;
        req[p::idx::INSTRUCTION] = p::INST_READ;
        req[p::idx::PARAM_0] = addr;
        req[p::idx::PARAM_0 + 1] = len;
        req[7] = !(id
            .wrapping_add(TX_LEN)
            .wrapping_add(p::INST_READ)
            .wrapping_add(addr)
            .wrapping_add(len));

        self.clear_rx_buffer().await;
        self.write_then_wait(&req, 80).await?;

        let total = 6 + len as usize; // FF FF ID LEN ERR data… CS
        let mut buf = [0u8; 32];
        self.read_exact_deadline(&mut buf[..total], Duration::from_millis(5))
            .await?;

        /* ---- validate ---- */
        if buf[..2] != p::HEADER || buf[p::idx::ID] != id {
            return Err(TxRxError::BadFormat);
        }
        let sum_ok = {
            let s = buf[2..total - 1]
                .iter()
                .fold(0u8, |a, &b| a.wrapping_add(b));
            !s == buf[total - 1]
        };
        if !sum_ok {
            return Err(TxRxError::BadChecksum);
        }

        let mut v = Vec::<u8, N>::new();
        v.extend_from_slice(&buf[p::idx::PARAM_0..p::idx::PARAM_0 + len as usize])
            .map_err(|_| TxRxError::BufferFull)?;
        Ok(v)
    }

    /** Write raw bytes to a register, wait for 6‑byte status. */
    async fn write_register(&mut self, id: u8, addr: u8, data: &[u8]) -> Result<(), TxRxError> {
        const BASE: u8 = 3; // INST + ADDR + CHK
        let len_field = BASE + data.len() as u8;

        let mut pkt = heapless::Vec::<u8, 16>::new();
        pkt.extend_from_slice(&p::HEADER).unwrap();
        pkt.extend_from_slice(&[id, len_field, p::INST_WRITE, addr])
            .unwrap();
        pkt.extend_from_slice(data).unwrap();
        let chk = !(pkt[2..].iter().fold(0u8, |a, &b| a.wrapping_add(b)));
        pkt.push(chk).unwrap();

        self.clear_rx_buffer().await;
        self.write_then_wait(&pkt, 80).await?;

        let mut status = [0u8; 6];
        self.read_exact_deadline(&mut status, Duration::from_millis(5))
            .await?;
        match status {
            [0xFF, 0xFF, rid, 0x02, err, cs] if rid == id => {
                let calc = !(rid.wrapping_add(0x02).wrapping_add(err));
                if calc != cs {
                    return Err(TxRxError::BadChecksum);
                }
                if err != 0 {
                    return Err(TxRxError::BadFormat);
                }
                Ok(())
            }
            _ => Err(TxRxError::BadFormat),
        }
    }

    /* ================= util conversions ===================== */
    pub fn counts_to_degrees(counts: u16, offset: f32) -> f32 {
        (counts as f32) * 360.0 / 4096.0 - offset
    }
    pub fn degrees_to_counts(deg: f32, offset: f32) -> u16 {
        ((deg + offset) * 4096.0 / 360.0) as u16
    }
}
