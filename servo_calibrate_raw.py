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


def move_to(bus, tick):
    tick = max(150, min(600, int(tick)))
    set_pwm(bus, CHANNEL, 0, tick)
    print(f"[MOVE] tick={tick}")
    return tick


def main():
    bus = SMBus(BUS)

    write8(bus, MODE1, ALLCALL)
    write8(bus, MODE2, OUTDRV)
    time.sleep(0.005)

    mode1 = read8(bus, MODE1)
    mode1 &= ~SLEEP
    write8(bus, MODE1, mode1)
    time.sleep(0.005)

    set_pwm_freq(bus, FREQ)

    tick = 307
    tick = move_to(bus, tick)

    print("")
    print("=== RAW TICK CALIBRATION ===")
    print("j : -1 tick")
    print("k : +1 tick")
    print("u : -5 tick")
    print("i : +5 tick")
    print("n : -20 tick")
    print("m : +20 tick")
    print("c : center(307)")
    print("q : quit")
    print("range clamp: 150 ~ 600")
    print("")

    try:
        while True:
            cmd = input("cmd> ").strip().lower()
            if cmd == 'j':
                tick = move_to(bus, tick - 1)
            elif cmd == 'k':
                tick = move_to(bus, tick + 1)
            elif cmd == 'u':
                tick = move_to(bus, tick - 5)
            elif cmd == 'i':
                tick = move_to(bus, tick + 5)
            elif cmd == 'n':
                tick = move_to(bus, tick - 20)
            elif cmd == 'm':
                tick = move_to(bus, tick + 20)
            elif cmd == 'c':
                tick = move_to(bus, 307)
            elif cmd == 'q':
                break
            else:
                print("unknown command")
    finally:
        move_to(bus, 307)
        time.sleep(0.3)
        bus.close()
        print("done")


if __name__ == "__main__":
    main()
