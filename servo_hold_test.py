import time
from smbus2 import SMBus

ADDR = 0x40
BUS = 7
CHANNEL = 0
FREQ = 50.0

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
    prescaleval = 25000000.0 / 4096.0 / freq_hz - 1.0
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

bus = SMBus(BUS)

write8(bus, MODE1, ALLCALL)
write8(bus, MODE2, OUTDRV)
time.sleep(0.005)

mode1 = read8(bus, MODE1)
mode1 &= ~SLEEP
write8(bus, MODE1, mode1)
time.sleep(0.005)

set_pwm_freq(bus, FREQ)

print("CENTER 323")
set_pwm(bus, CHANNEL, 0, 323)
time.sleep(3.0)

print("LEFT 240")
set_pwm(bus, CHANNEL, 0, 240)
time.sleep(3.0)

print("RIGHT 400")
set_pwm(bus, CHANNEL, 0, 400)
time.sleep(3.0)

print("CENTER 323")
set_pwm(bus, CHANNEL, 0, 323)
time.sleep(3.0)

bus.close()
print("done")
