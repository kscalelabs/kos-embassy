#!/usr/bin/env python3
"""
SPI DSR Hardware Handshaking Test
─────────────────────────────────────────────────────────────────────────────
• Uses DSR (GPIO 23) for hardware flow control
• Protocol:
  1. Wait for DSR LOW (STM32 ready to receive)
  2. Send command to STM32
  3. Wait for DSR HIGH (STM32 response ready)
  4. Clock NOOP to receive response
  5. Wait for DSR LOW (ready for next command)
• 260-byte frames with CRC16 validation
"""

import time
import spidev
import struct
import RPi.GPIO as GPIO
import os
import errno
import os
import psutil

def cpu_affinity():
    """Pin process to isolated CPU core and move IRQs away"""
    try:
        # Pin this process to CPU core 3 (assuming 4-core Pi)
        os.sched_setaffinity(0, {3})
        print("✓ Pinned to CPU core 3")
        
    except Exception as e:
        print(f"⚠ CPU affinity failed: {e}")

def set_realtime(priority: int = 80):
    """Set process to real-time priority"""
    SCHED_FIFO = 1
    param = os.sched_param(priority)
    try:
        os.sched_setscheduler(0, SCHED_FIFO, param)
        print(f"✓ Set real-time priority: {priority}")
    except PermissionError as e:
        if e.errno == errno.EPERM:
            print("⚠ Need sudo for RT priority, continuing at normal priority.")
        else:
            raise

set_realtime(80)
cpu_affinity()

# ───────── Configuration ─────────────────────────────────────────
SPI_BUS   = 0
SPI_DEV   = 0
SPI_MODE  = 0
SCK_HZ    = 16_000_000      # 16 MHz for STM32-G0 slave
DSR_PIN   = 23              # GPIO 23 = DSR from STM32
PAYLOAD   = b"Hello STM32!" # Test payload
TIMEOUT_S = 1.0             # DSR transition timeout
# ──────────────────────────────────────────────────────────────────

FRAME_LEN = 260
SYNC      = b"\xAA\x55"
CRC16_POLY = 0x1021

def crc16_ccitt(buf: bytes, crc=0xFFFF) -> int:
    """Calculate CRC16-CCITT"""
    for b in buf:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ CRC16_POLY) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def build_command(seq: int, payload: bytes) -> bytes:
    """Build a command frame with header, payload, and CRC"""
    payload_len = len(payload)
    frame = bytearray(5 + payload_len + 2)  # header + payload + CRC
    
    # Header: SYNC (2) + LEN (1) + SEQ (1) + CMD (1)
    struct.pack_into("<HBB", frame, 0, 0x55AA, payload_len, seq & 0xFF)
    frame[4] = 0x01  # Command type (arbitrary)
    
    # Payload
    frame[5:5+payload_len] = payload
    
    # CRC
    crc = crc16_ccitt(frame[:5+payload_len])
    struct.pack_into("<H", frame, 5+payload_len, crc)
    
    # Pad to frame length
    frame.extend(b"\0" * (FRAME_LEN - len(frame)))
    return bytes(frame)

def wait_for_dsr(state: bool, timeout: float = TIMEOUT_S) -> bool:
    """Wait for DSR to reach desired state. Returns True if achieved, False if timeout"""
    start_time = time.perf_counter()
    while (time.perf_counter() - start_time) < timeout:
        if GPIO.input(DSR_PIN) == state:
            return True
    return False

def validate_response(rx_frame: bytes, expected_seq: int) -> tuple[bool, str]:
    """Validate received response frame"""
    if len(rx_frame) < 7:
        return False, "Frame too short"
    
    if rx_frame[0:2] != SYNC:
        return False, f"Invalid sync: {rx_frame[0]:02X} {rx_frame[1]:02X}"
    
    payload_len = rx_frame[2]
    if payload_len > 256:
        return False, f"Invalid payload length: {payload_len}"
    
    if len(rx_frame) < 5 + payload_len + 2:
        return False, "Frame truncated"
    
    # Check CRC
    crc_pos = 5 + payload_len
    received_crc = struct.unpack("<H", rx_frame[crc_pos:crc_pos+2])[0]
    calculated_crc = crc16_ccitt(rx_frame[:crc_pos])
    
    if received_crc != calculated_crc:
        return False, f"CRC mismatch: got {received_crc:04X}, expected {calculated_crc:04X}"
    
    # Check sequence (should be echoed by STM32)
    response_seq = rx_frame[3]
    expected_response_seq = (expected_seq + 1) & 0xFF
    if response_seq != expected_response_seq:
        print(f"Sequence mismatch: got {response_seq}, expected {expected_response_seq}")
        expected_response_seq = response_seq
    
    return True, "OK"


# ───────── Setup ───────────────────────────────────────────────
GPIO.setmode(GPIO.BCM)
GPIO.setup(DSR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEV)
spi.mode = SPI_MODE
spi.max_speed_hz = SCK_HZ
spi.no_cs = False

print(f"SPI: Bus {SPI_BUS}.{SPI_DEV}, Mode {SPI_MODE}, {SCK_HZ/1e6:.1f} MHz")
print(f"DSR: GPIO {DSR_PIN}")
print(f"Frame: {FRAME_LEN} bytes, Payload: {len(PAYLOAD)} bytes")
print()

try:
    seq = 0
    success_count = 0
    total_count = 0
    recovery_count = 0
    stuck_high_count = 0
    resync_count = 0
    
    print("Starting Echo Test with DSR handshaking (Ctrl+C to stop)...")
    
    while True:
        time.sleep(0.01)
        total_count += 1
        
        print(f"[{total_count:3d}] Seq {seq:02X}: ", end="", flush=True)
        
        # Step 1: Wait for DSR LOW (ready to receive)
        if not wait_for_dsr(False, TIMEOUT_S):
            print("TIMEOUT waiting for DSR LOW")
            continue
        
        # Step 2: Send command
        cmd_frame = build_command(seq, PAYLOAD)
        t0 = time.perf_counter()
        spi.xfer2(cmd_frame)  # 260 bytes
        cmd_time = (time.perf_counter() - t0) * 1000
        print(f"CMD sent ({cmd_time:.1f}ms) → ", end="", flush=True)
        
        # Step 3: Wait for DSR HIGH (response ready)
        if not wait_for_dsr(True, TIMEOUT_S):
            print("TIMEOUT waiting for DSR HIGH")
            seq = (seq + 1) & 0xFF
            continue
        
        # Step 4: Clock out response with NOOP
        noop_frame = bytes(FRAME_LEN)
        t0 = time.perf_counter()
        response = bytes(spi.xfer2(noop_frame))  # 260 bytes
        resp_time = (time.perf_counter() - t0) * 1000
        
        # Step 5: Validate response
        is_valid, error_msg = validate_response(response, seq)
        
        if is_valid:
            success_count += 1
            print(f"RESP OK ({resp_time:.1f}ms)")
        else:
            print(f"{error_msg}")
        
        print(f"RTT: {resp_time+cmd_time:.1f}ms")
        seq = (seq + 1) & 0xFF

except KeyboardInterrupt:
    print("\n\nTest stopped by user")
    
except Exception as e:
    print(f"\nError: {e}")
    
finally:
    spi.close()
    GPIO.cleanup()
    
    if total_count > 0:
        success_rate = (success_count / total_count) * 100
        print(f"\n Summary:")
        print(f"   Total transactions: {total_count}")
        print(f"   Successful: {success_count}")
        print(f"   Success rate: {success_rate:.1f}%")