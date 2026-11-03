"""
MicroPython driver for the MS5837-30BA pressure/depth sensor over I2C.

Tested with the Blu-Sub Subsea M8 Depth Sensor (0-30 Bar / 0-300 m),
part BS-SSC-DPTHS-BLK-08A1, which packages the MS5837-30BA chip in a
waterproof M8 housing. The Blue Robotics Bar30 uses the same chip and
works with this driver unchanged.

Requires an I2C bus (3.3 V logic). Supply VIN at 5 V.
Provides read(), pressure(), temperature(), and depth() methods.

Example usage on Pico W:

    from machine import I2C, Pin
    import Depth_Sensor, time

    i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
    sensor = Depth_Sensor.MS5837(i2c)

    while True:
        sensor.read()
        print(sensor.depth(), 'm', sensor.temperature(), 'C')
        time.sleep_ms(200)

"""

from micropython import const
import time


_MS5837_ADDR        = const(0x76)

_CMD_RESET          = const(0x1E)
_CMD_ADC_READ       = const(0x00)
_CMD_PROM_READ      = const(0xA0)   # base address; add 2*n for coefficient n (0..6)

# Convert D1 (pressure) - one command per OSR setting
_CMD_CONVERT_D1     = const(0x40)   # add 2*osr_index for higher OSR
# Convert D2 (temperature)
_CMD_CONVERT_D2     = const(0x50)   # add 2*osr_index for higher OSR

# OSR index: 0=256, 1=512, 2=1024, 3=2048, 4=4096, 5=8192
# Max conversion time per datasheet (ms), with 1 ms margin
_CONV_TIME_MS       = (1, 2, 3, 5, 10, 20)

# Fluid densities (kg/m^3) for converting pressure -> depth
DENSITY_FRESHWATER  = 997
DENSITY_SALTWATER   = 1029

_P_ATM_MBAR         = 1013.25       # standard atmospheric pressure
_G                  = 9.80665       # m/s^2


def _crc4(prom):
    # CRC4 check per MS5837-30BA datasheet. `prom` is a list of 7 uint16.
    n_rem = 0
    crc_read = prom[0]
    prom[0] = prom[0] & 0x0FFF      # strip CRC nibble from word 0
    prom.append(0)                  # virtual trailing byte
    for cnt in range(16):
        if cnt & 1:
            n_rem ^= prom[cnt >> 1] & 0x00FF
        else:
            n_rem ^= prom[cnt >> 1] >> 8
        for _ in range(8):
            if n_rem & 0x8000:
                n_rem = ((n_rem << 1) ^ 0x3000) & 0xFFFF
            else:
                n_rem = (n_rem << 1) & 0xFFFF
    prom.pop()
    prom[0] = crc_read              # restore for debugging
    return (n_rem >> 12) & 0x000F


class MS5837:
    def __init__(self, i2c, addr=_MS5837_ADDR, osr=5):
        self.i2c = i2c
        self.addr = addr
        # OSR index 0..5. 5 = 8192 samples, ~20 ms per conversion, best accuracy.
        # Drop to 3 or 4 if you need faster sampling for velocity calc.
        self.osr = osr

        self._buf2 = bytearray(2)
        self._buf3 = bytearray(3)
        self._C = [0] * 7           # PROM calibration coefficients C0..C6
        self._D1 = 0                # raw pressure ADC
        self._D2 = 0                # raw temperature ADC
        self._pressure_mbar = 0.0
        self._temperature_c = 0.0
        self.fluid_density = DENSITY_FRESHWATER

        self.init_sensor()

    def init_sensor(self):
        # Reset; datasheet requires >= 2.8 ms before next command
        try:
            self.i2c.writeto(self.addr, b"\x1e")
        except OSError:
            raise OSError("no MS5837 at I2C 0x%02x" % self.addr)
        time.sleep_ms(10)

        # Read 7 PROM coefficients (each 16-bit big-endian)
        for i in range(7):
            self.i2c.writeto(self.addr, bytes([_CMD_PROM_READ + (i * 2)]))
            self.i2c.readfrom_into(self.addr, self._buf2)
            self._C[i] = (self._buf2[0] << 8) | self._buf2[1]

        # Verify factory CRC
        crc_expected = (self._C[0] >> 12) & 0x000F
        crc_calc = _crc4(list(self._C))
        if crc_expected != crc_calc:
            raise OSError("MS5837 PROM CRC mismatch (got %d, want %d)"
                          % (crc_calc, crc_expected))

    def _convert(self, base_cmd):
        # Trigger ADC conversion, wait, then read 24-bit result
        self.i2c.writeto(self.addr, bytes([base_cmd + (self.osr * 2)]))
        time.sleep_ms(_CONV_TIME_MS[self.osr])
        self.i2c.writeto(self.addr, b"\x00")
        self.i2c.readfrom_into(self.addr, self._buf3)
        return (self._buf3[0] << 16) | (self._buf3[1] << 8) | self._buf3[2]

    def read(self):
        # Acquire raw D1 (pressure) and D2 (temperature)
        self._D1 = self._convert(_CMD_CONVERT_D1)
        self._D2 = self._convert(_CMD_CONVERT_D2)

        C = self._C

        # First-order compensation (MS5837-30BA datasheet)
        dT   = self._D2 - (C[5] << 8)
        TEMP = 2000 + ((dT * C[6]) >> 23)

        OFF  = (C[2] << 16) + ((C[4] * dT) >> 7)
        SENS = (C[1] << 15) + ((C[3] * dT) >> 8)

        # Second-order temperature compensation
        if TEMP < 2000:
            Ti    = (3 * (dT * dT)) >> 33
            OFFi  = (3 * ((TEMP - 2000) ** 2)) >> 1
            SENSi = (5 * ((TEMP - 2000) ** 2)) >> 3
            if TEMP < -1500:
                OFFi  += 7 * ((TEMP + 1500) ** 2)
                SENSi += 4 * ((TEMP + 1500) ** 2)
        else:
            Ti    = (2 * (dT * dT)) >> 37
            OFFi  = ((TEMP - 2000) ** 2) >> 4
            SENSi = 0

        OFF2  = OFF  - OFFi
        SENS2 = SENS - SENSi
        TEMP2 = TEMP - Ti

        # P is in 0.1 mbar units; TEMP2 is in 0.01 deg C
        P = (((self._D1 * SENS2) >> 21) - OFF2) >> 13

        self._pressure_mbar = P / 10.0
        self._temperature_c = TEMP2 / 100.0

    # -- accessors ----------------------------------------------------

    def pressure(self):
        # mbar (= hPa)
        return self._pressure_mbar

    def pressure_pa(self):
        return self._pressure_mbar * 100.0

    def temperature(self):
        # degrees Celsius
        return self._temperature_c

    def depth(self):
        # Depth in meters from gauge pressure: (P - P_atm) / (rho * g)
        return ((self._pressure_mbar - _P_ATM_MBAR) * 100.0
                / (self.fluid_density * _G))

    def altitude(self):
        # Above-sea-level altitude (m) when used in air, for sanity tests
        return (1 - (self._pressure_mbar / _P_ATM_MBAR) ** 0.190284) * 44307.69

    # -- config -------------------------------------------------------

    def set_fluid_density(self, density_kg_m3):
        # Pool water is between fresh (997) and salt (1029); set 1000 for
        # typical chlorinated pool water.
        self.fluid_density = density_kg_m3

    def set_osr(self, osr):
        # 0..5. Higher = more accurate, slower per read.
        if not 0 <= osr <= 5:
            raise ValueError("osr must be 0..5")
        self.osr = osr
