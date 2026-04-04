import time
from smbus2 import SMBus

ADDR = 0x40
BUS = 7

MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

RESTART = 0x80
SLEEP = 0x10
ALLCALL = 0x01
OUTDRV = 0x04

def write8(bus, reg, value):
    bus.write_byte_data(ADDR, reg, value & 0xFF)

def read8(bus, reg):
    return bus.read_byte_data(ADDR, reg)

def set_pwm_freq(bus, freq_hz):
    prescaleval = 25000000.0
    prescaleval /= 4096.0
    prescaleval /= float(freq_hz)
    prescaleval -= 1.0
    prescale = int(round(prescaleval))

    oldmode = read8(bus, MODE1)
    newmode = (oldmode & 0x7F) | SLEEP
    write8(bus, MODE1, newmode)
    write8(bus, PRESCALE, prescale)
    write8(bus, MODE1, oldmode)
    time.sleep(0.005)
    write8(bus, MODE1, oldmode | RESTART)

def set_pwm(bus, channel, on, off):
    reg = LED0_ON_L + 4 * channel
    write8(bus, reg, on & 0xFF)
    write8(bus, reg + 1, (on >> 8) & 0xFF)
    write8(bus, reg + 2, off & 0xFF)
    write8(bus, reg + 3, (off >> 8) & 0xFF)

def set_pulse_us(bus, channel, pulse_us, freq=50.0):
    period_us = 1_000_000.0 / freq
    ticks = int((pulse_us / period_us) * 4096.0)
    ticks = max(0, min(4095, ticks))
    set_pwm(bus, channel, 0, ticks)

def set_angle(bus, channel, angle_deg, min_us=400.0, max_us=2600.0):
    angle_deg = max(0.0, min(180.0, angle_deg))
    pulse = min_us + (angle_deg / 180.0) * (max_us - min_us)
    set_pulse_us(bus, channel, pulse, freq=50.0)

bus = SMBus(BUS)

write8(bus, MODE1, ALLCALL)
write8(bus, MODE2, OUTDRV)
time.sleep(0.005)

mode1 = read8(bus, MODE1)
mode1 = mode1 & ~SLEEP
write8(bus, MODE1, mode1)
time.sleep(0.005)

set_pwm_freq(bus, 50.0)

print("90 deg")
set_angle(bus, 0, 90)
time.sleep(1.5)

print("70 deg")
set_angle(bus, 0, 70)
time.sleep(1.5)

print("110 deg")
set_angle(bus, 0, 110)
time.sleep(1.5)

print("90 deg")
set_angle(bus, 0, 90)
time.sleep(1.5)

bus.close()
print("done")
