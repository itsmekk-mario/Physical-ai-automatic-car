import time
from smbus2 import SMBus

PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_PRESCALE = 0xFE
LED0_ON_L = 0x06

RESTART = 0x80
SLEEP = 0x10
ALLCALL = 0x01
OUTDRV = 0x04


class PCA9685:
    def __init__(self, bus_num=7, address=0x40, pwm_freq=50.0):
        self.bus_num = bus_num
        self.address = address
        self.bus = SMBus(bus_num)
        self.pwm_freq = pwm_freq

        self.write8(PCA9685_MODE1, ALLCALL)
        self.write8(PCA9685_MODE2, OUTDRV)
        time.sleep(0.005)

        mode1 = self.read8(PCA9685_MODE1)
        mode1 = mode1 & ~SLEEP
        self.write8(PCA9685_MODE1, mode1)
        time.sleep(0.005)

        self.set_pwm_freq(pwm_freq)

    def close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None

    def read8(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def write8(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def set_pwm_freq(self, freq_hz):
        prescaleval = 25000000.0
        prescaleval /= 4096.0
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        prescale = int(round(prescaleval))

        oldmode = self.read8(PCA9685_MODE1)
        newmode = (oldmode & 0x7F) | SLEEP
        self.write8(PCA9685_MODE1, newmode)
        self.write8(PCA9685_PRESCALE, prescale)
        self.write8(PCA9685_MODE1, oldmode)
        time.sleep(0.005)
        self.write8(PCA9685_MODE1, oldmode | RESTART)

    def set_pwm(self, channel, on, off):
        reg = LED0_ON_L + 4 * channel
        self.write8(reg, on & 0xFF)
        self.write8(reg + 1, (on >> 8) & 0xFF)
        self.write8(reg + 2, off & 0xFF)
        self.write8(reg + 3, (off >> 8) & 0xFF)

    def set_duty_cycle(self, channel, duty):
        duty = max(0.0, min(1.0, duty))
        off = int(duty * 4095)
        self.set_pwm(channel, 0, off)