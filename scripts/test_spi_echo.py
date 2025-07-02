#!/usr/bin/env python3
"""
200 Hz SPI ping-pong test – 260-byte self-sync frame
────────────────────────────────────────────────────
• One 260-byte full-duplex transfer every 5 ms
• Uses hardware chip select CE0
• Prints OK-rate and RTT µs stats once a second
"""

import time, spidev, struct, statistics
import os, ctypes, errno

def set_realtime(priority: int = 80):
    """
    Put the current process into SCHED_FIFO with the given priority.
    Needs CAP_SYS_NICE (run with sudo or give the binary that capability).
    """
    SCHED_FIFO = 1                                  # Linux constant
    param = os.sched_param(priority)
    try:
        os.sched_setscheduler(0, SCHED_FIFO, param) # 0 = this process
    except PermissionError as e:
        if e.errno == errno.EPERM:
            print("⚠  Need sudo or CAP_SYS_NICE to set RT priority,"
                  " continuing at normal nice level.")
        else:
            raise

set_realtime(80)

# ───────── user parameters ──────────────────────────────────────
SPI_BUS   = 0
SPI_DEV   = 0
SPI_MODE  = 0
SCK_HZ    = 16_000_000       # safe for STM32-G0 as slave
RATE_HZ   = 200
PAYLOAD   = b"A"*10          # tweak length if you like
# ----------------------------------------------------------------

FRAME_LEN = 260              # fixed DMA chunk in firmware
SYNC      = b"\xAA\x55"
CRC16_POLY = 0x1021

def crc16_ccitt(buf: bytes, crc=0xFFFF) -> int:
    for b in buf:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ CRC16_POLY) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

# open SPI -------------------------------------------------------
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEV)
spi.mode = SPI_MODE
spi.max_speed_hz = SCK_HZ
spi.no_cs = False            # hardware CE0

# pre-allocate buffers ------------------------------------------
blank_frame = bytes(FRAME_LEN)        # for first dummy cycle
seq         = 0                       # command sequence number

period      = 1.0 / RATE_HZ
deadline    = time.perf_counter()

ok_cnt = total_cnt = 0
rtt_min = float("inf"); rtt_max = 0.0; rtt_sum = 0.0
next_report = deadline + 1.0

prev_cmd  = blank_frame               # seed; first reply is junk

# Build a new TX frame for the given seq ------------------------
def build_cmd(seq: int) -> bytes:
    payload_len = len(PAYLOAD)
    frame = bytearray(5 + payload_len + 2)   # hdr+payload+CRC
    struct.pack_into("<HB", frame, 0, 0x55AA, payload_len)  # SYNC+LEN
    frame[3] = seq & 0xFF
    frame[4] = 0x01                         # CMD (arbitrary)
    frame[5:5+payload_len] = PAYLOAD
    crc = crc16_ccitt(frame[:5+payload_len])
    struct.pack_into("<H", frame, 5+payload_len, crc)
    frame.extend(b"\0" * (FRAME_LEN - len(frame)))  # pad to 260 B
    return bytes(frame)

try:
    while True:
        # pacing -----------------------------------------------------------
        now = time.perf_counter()
        sleep_left = deadline - now
        if sleep_left > 0:
            time.sleep(sleep_left)
        deadline += period

        cmd = build_cmd(seq)
        t0  = time.perf_counter()
        rx  = bytes(spi.xfer2(cmd))             # 260-byte full-duplex
        rtt = (time.perf_counter() - t0) * 1e6  # µs

        # -------- verify reply of previous command ------------------------
        if total_cnt > 0:                     # skip first dummy cycle
            if rx[0:2] == SYNC:
                pay_len = rx[2]
                seq_rx  = rx[3]

                crc_ok  = crc16_ccitt(rx[:5+pay_len]) == int.from_bytes(rx[5+pay_len:7+pay_len], "little")
                seq_ok  = seq_rx == ((seq - 1) & 0xFF)
                if crc_ok and seq_ok:
                    ok_cnt += 1
                    rtt_min = min(rtt_min, rtt)
                    rtt_max = max(rtt_max, rtt)
                    rtt_sum += rtt
        total_cnt += 1
        seq = (seq + 1) & 0xFF

        # -------- once-per-second stats print -----------------------------
        now = time.perf_counter()
        if now >= next_report:
            if ok_cnt:
                rtt_avg = rtt_sum / ok_cnt
                pct_ok  = ok_cnt / (total_cnt - 1) * 100   # minus first dummy
                print(f"[{int(next_report)}s] OK {ok_cnt}/{total_cnt-1}"
                      f" ({pct_ok:5.1f} %)   RATE {RATE_HZ} Hz, "
                      f"RTT µs min/avg/max = "
                      f"{rtt_min:6.0f}/{rtt_avg:6.0f}/{rtt_max:6.0f}")
            else:
                print(f"[{int(next_report)}s] OK 0/{total_cnt-1} (  0.0 %)")
            next_report += 2.0
except KeyboardInterrupt:
    pass
finally:
    spi.close()