"""
Oceanus float controller (Raspberry Pi Pico W / Pico 2 W, MicroPython).

Copy to the Pico as main.py, together with float_logic.py,
Depth_Sensor.py and sdcard.py. The decision logic lives in
float_logic.py and is desk-tested; this file is only the hardware glue.
See README.md and the Float Software Design Document.

Pins (Drive drawing "Float Pico W Connections"):
    UART0 GP0 TX / GP1 RX   ultrasonic sensor (via 5 V-3.3 V level shifter)
    I2C0  GP8 SDA / GP9 SCL MS5837-30BA pressure sensor, address 0x76
    SPI0  GP18 SCK / GP19 MOSI / GP16 MISO / GP17 CS   SD card
    GP14 PWM                peristaltic pump speed (white wire)
    GP15                    peristaltic pump direction (green wire)
    GP27 ADC1               thermistor divider (diagnostic log only)
"""

from machine import Pin, UART, I2C, SPI, ADC, PWM, WDT
from micropython import const
import bluetooth
import json
import os
import time
import vfs

import sdcard
import Depth_Sensor
import float_logic as fl

CONTROL_PERIOD_MS = 250
LOG_PERIOD_MS = int(fl.LOG_PERIOD_S * 1000)
ULTRASONIC_TIMEOUT_MS = 150
ULTRASONIC_MIN_MM = 30          # [TBD: blind zone of the fitted sensor]
ULTRASONIC_MAX_MM = 6000
ULTRASONIC_STALE_MS = 1000      # older than this counts as no reading
SOUND_SCALE = 1.0               # [TBD: 1480/343 if the sensor is calibrated for air]
PUMP_IN_IS_FORWARD = True       # [TBD: confirm on the bench which way floods the reservoir]
PUMP_STROKE_S = None            # [TBD: seconds at 100 % for a full stroke]
ZERO_SAMPLES = 20
STATE_PATH = "/float_state.json"    # internal flash, survives a dead SD card

SERVICE_UUID = bluetooth.UUID("ABCDABCD-1234-ABCD-BBBB-123412341234")
TX_UUID = bluetooth.UUID("ABCDABCD-1234-ABDD-CCCC-123412341235")   # float -> station (notify)
RX_UUID = bluetooth.UUID("ABCDABCD-1234-ABDD-CCCC-123412341236")   # station -> float (write)
_IRQ_CONNECT = const(1)
_IRQ_DISCONNECT = const(2)
_IRQ_WRITE = const(3)
_FLAG_READ = const(0x0002)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)
BLE_CHUNK = 20                  # default ATT MTU 23 - 3


### Link to the mission station (GNC-FL-01, FL-02) ###
class Link:
    """BLE peripheral. Everything sent is a stream of text lines, split
    into 20-byte notifications. Commands from the station:
      T<unix seconds>  set the clock to UTC
      GO               start the descent (only after a packet was sent)
      DUMP             send the stored packet log, then EOF"""

    def __init__(self, name="Oceanus"):
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        ((self.tx, self.rx),) = self.ble.gatts_register_services((
            (SERVICE_UUID, ((TX_UUID, _FLAG_READ | _FLAG_NOTIFY),
                            (RX_UUID, _FLAG_WRITE))),
        ))
        self.ble.config(gap_name=name)
        self._adv = self._payload(name)
        self.conn = None
        self.commands = []
        self.ble.irq(self._irq)
        self.advertise()

    @staticmethod
    def _payload(name):
        p = bytearray(b"\x02\x01\x06")
        n = name.encode()
        p += bytes((len(n) + 1, 0x09)) + n
        u = bytes(reversed(bytes(SERVICE_UUID)))
        p += bytes((len(u) + 1, 0x07)) + u
        return p

    def advertise(self):
        self.ble.gap_advertise(100000, self._adv)   # 100 ms interval

    def _irq(self, event, data):
        if event == _IRQ_CONNECT:
            self.conn = data[0]
        elif event == _IRQ_DISCONNECT:
            self.conn = None
            self.advertise()
        elif event == _IRQ_WRITE:
            self.commands.append(self.ble.gatts_read(self.rx).decode().strip())

    def connected(self):
        return self.conn is not None

    def send_line(self, line):
        if self.conn is None:
            return False
        data = (line + "\n").encode()
        try:
            for i in range(0, len(data), BLE_CHUNK):
                self.ble.gatts_notify(self.conn, self.tx, data[i:i + BLE_CHUNK])
                time.sleep_ms(15)
            return True
        except OSError:
            return False

    def send_file(self, path, wdt=None):
        self.send_line("BEGIN LOG")
        try:
            with open(path) as f:
                for line in f:
                    if not self.send_line(line.rstrip()):
                        return False
                    if wdt:
                        wdt.feed()
        except OSError:
            self.send_line("NO LOG")
        return self.send_line("EOF")


### Sensors and actuator ###
def read_ultrasonic(uart):
    """Trigger and read one frame; distance to whatever is overhead in m,
    or None. Never blocks longer than ULTRASONIC_TIMEOUT_MS."""
    uart.read()                 # flush stale bytes
    uart.write(b"\x55")         # trigger (ignored by auto-output sensors)
    t0 = time.ticks_ms()
    while uart.any() < 4:
        if time.ticks_diff(time.ticks_ms(), t0) > ULTRASONIC_TIMEOUT_MS:
            return None
    # Sync to the 0xFF header if the sensor was mid-frame
    while uart.any() >= 4:
        b = uart.read(1)
        if b and b[0] == 0xFF:
            mm = fl.parse_ultrasonic_frame(b + (uart.read(3) or b""))
            if mm is None or not (ULTRASONIC_MIN_MM <= mm <= ULTRASONIC_MAX_MM):
                return None
            return mm / 1000.0 * SOUND_SCALE
    return None


class Pump:
    """Peristaltic pump. cmd in %: positive floods (sink), negative empties (rise)."""

    def __init__(self, speed_pin=14, dir_pin=15):
        self.pwm = PWM(Pin(speed_pin))
        self.pwm.freq(20000)
        self.dir = Pin(dir_pin, Pin.OUT, value=0)
        self.stop()

    def set(self, cmd):
        forward = (cmd > 0) == PUMP_IN_IS_FORWARD
        if forward:
            self.dir.init(Pin.OUT)
            self.dir.value(0)
        else:
            self.dir.init(Pin.IN)
        duty = abs(cmd)
        self.pwm.duty_u16(0 if duty < fl.MIN_DUTY else int(min(duty, 100) / 100 * 65535))

    def stop(self):
        self.pwm.duty_u16(0)


def mount_sd(spi, cs):
    try:
        vfs.mount(vfs.VfsFat(sdcard.SDCard(spi, cs)), "/sd")
        return "/sd"
    except Exception as e:
        print("SD card error:", e, "- logging to flash")
        return ""


def load_state():
    try:
        with open(STATE_PATH) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def save_state(d):
    try:
        with open(STATE_PATH, "w") as f:
            json.dump(d, f)
    except OSError as e:
        print("state save failed:", e)


### Main ###
def main():
    # Advertise first so the station can connect within 5 s (GNC-FL-01).
    link = Link()
    pump = Pump()
    uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
    i2c = I2C(0, sda=Pin(8), scl=Pin(9), freq=400000)
    spi = SPI(0, baudrate=1000000, polarity=0, phase=0,
              sck=Pin(18), mosi=Pin(19), miso=Pin(16))
    thermistor = ADC(27)
    root = mount_sd(spi, Pin(17, Pin.OUT, value=1))
    packets_path = root + "/packets.txt"
    diag_path = root + "/diag.csv"

    sensor = Depth_Sensor.MS5837(i2c, osr=4)    # depth is computed from the surface zero, not sensor.depth()

    def pressure_pa():
        sensor.read()
        return sensor.pressure_pa()

    # Surface zero, or resume after a brownout underwater.
    saved = load_state()
    zero, seg = fl.resume_decision(saved, pressure_pa())
    mission = fl.Mission()
    if zero is None:
        zero = fl.surface_zero([pressure_pa() for _ in range(ZERO_SAMPLES)])
        stroke = fl.StrokeTracker(PUMP_STROKE_S)
        print("surface zero {:.0f} Pa".format(zero))
    else:
        mission.start(seg)
        stroke = fl.StrokeTracker(PUMP_STROKE_S, saved.get("stroke_pos", 0.0))
        print("brownout resume at", mission.state)
    state = {"zero_pa": zero, "seg": mission.seg, "stroke_pos": stroke.pos}
    save_state(state)

    wdt = WDT(timeout=8000)     # a hang reboots into the resume path above
    vel = fl.VelocityEstimator()
    utc_offset = None           # unix seconds at boot, once the station syncs
    packet_sent = False
    dump_pending = False
    was_connected = False
    last_range = None
    last_range_ms = time.ticks_ms()
    next_log = time.ticks_add(time.ticks_ms(), LOG_PERIOD_MS)
    last_ms = time.ticks_ms()

    pf = open(packets_path, "a")
    df = open(diag_path, "a")
    df.write("t_s,state,depth_m,vel_mps,ice_m,cmd_pct,hold_s,therm_raw\n")

    while True:
        loop_start = time.ticks_ms()
        wdt.feed()
        t = loop_start / 1000.0
        dt = time.ticks_diff(loop_start, last_ms) / 1000.0
        last_ms = loop_start

        p = pressure_pa()
        depth = fl.depth_from_pressure(p, zero)
        v = vel.update(t, depth)
        r = read_ultrasonic(uart)
        if r is not None:
            last_range, last_range_ms = r, loop_start
        elif time.ticks_diff(loop_start, last_range_ms) > ULTRASONIC_STALE_MS:
            last_range = None   # no reading: float_logic assumes ice

        # Station commands
        while link.commands:
            c = link.commands.pop(0)
            if c.startswith("T"):
                try:
                    utc_offset = int(c[1:]) - time.ticks_ms() // 1000
                except ValueError:
                    pass
            elif c == "GO" and mission.state == "PREDIVE":
                if packet_sent:
                    mission.start()
                    link.send_line("DESCENDING")
                else:
                    link.send_line("REFUSED: no packet sent yet")
            elif c == "DUMP":
                dump_pending = True
        if link.connected() and not was_connected and mission.state == "END":
            dump_pending = True     # recovered: send the log on connect
        was_connected = link.connected()

        cmd, event = mission.update(t, depth, v, last_range)
        cmd = stroke.limit(cmd)
        pump.set(cmd)
        stroke.update(cmd, dt)
        if event:
            print(event, "complete")
            state["seg"] = mission.seg
            state["stroke_pos"] = stroke.pos
            save_state(state)

        # 1 Hz log (Calc C6), to SD and live over the link when connected.
        if time.ticks_diff(loop_start, next_log) >= 0:
            next_log = time.ticks_add(next_log, LOG_PERIOD_MS)
            now_s = time.ticks_ms() // 1000
            synced = utc_offset is not None
            packet = fl.format_packet(now_s + utc_offset if synced else now_s,
                                      p - zero, depth, utc=synced)
            try:
                pf.write(packet + "\n")
                pf.flush()
                hold = mission.timer.elapsed(t) if mission.timer else 0.0
                df.write("{:.1f},{},{:.3f},{:.3f},{},{:.0f},{:.1f},{}\n".format(
                    t, mission.state, depth, v,
                    "" if last_range is None else "{:.2f}".format(last_range),
                    cmd, hold, thermistor.read_u16()))
                df.flush()
            except OSError as e:
                print("log write failed:", e)
            if link.send_line(packet):
                packet_sent = True
            print(packet, mission.state)
            state["stroke_pos"] = stroke.pos
            save_state(state)

        if dump_pending and link.connected():
            dump_pending = False
            pf.flush()
            if not link.send_file(packets_path, wdt):
                dump_pending = True     # link dropped mid-dump; resend on reconnect

        spent = time.ticks_diff(time.ticks_ms(), loop_start)
        if spent < CONTROL_PERIOD_MS:
            time.sleep_ms(CONTROL_PERIOD_MS - spent)


try:
    main()
finally:
    # Never leave the pump running if the program stops.
    PWM(Pin(14)).duty_u16(0)
