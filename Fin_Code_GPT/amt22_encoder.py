"""
Module: AMT22 Encoder Driver
Author: He
Original Ref: encoder.cpp
Description: 
    Handles SPI communication (spidev) with the AMT22 absolute encoder. 
    Converts raw binary data into degrees to provide high-precision 
    position feedback for the PID controller.
"""
import time
import spidev
AMT22_NOP = 0x00
AMT22_RESET = 0x60
AMT22_ZERO = 0x70
DELAY = 0.000003

spi = spidev.SpiDev()


def encoder_init(bus=0, device=0):
    spi.open(bus, device)
    spi.max_speed_hz = 1000000
    spi.mode = 0


def send_cmd(cmd: int) -> int:
    high_byte = spi.xfer2([AMT22_NOP])[0]
    time.sleep(DELAY)
    low_byte = spi.xfer2([cmd])[0]
    time.sleep(DELAY)
    return (high_byte << 8) | low_byte


def encoder_reset():
    send_cmd(AMT22_RESET)
    time.sleep(0.2)


def encoder_set_zero():
    send_cmd(AMT22_ZERO)
    time.sleep(0.2)


def get_raw_position() -> int:
    current_pos = send_cmd(AMT22_NOP)
    b = [(current_pos >> i) & 0x01 for i in range(16)]

    odd_bit = not (b[13] ^ b[11] ^ b[9] ^ b[7] ^ b[5] ^ b[3] ^ b[1])
    even_bit = not (b[12] ^ b[10] ^ b[8] ^ b[6] ^ b[4] ^ b[2] ^ b[0])

    if b[15] == odd_bit and b[14] == even_bit:
        return current_pos & 0x3FFF

    return 0xFFFF


def get_position() -> float | None:
    raw = get_raw_position()

    if raw == 0xFFFF:
        return None

    pos_deg = (raw / 16384.0) * 360.0

    if pos_deg > 180:
        pos_deg -= 360

    return pos_deg
