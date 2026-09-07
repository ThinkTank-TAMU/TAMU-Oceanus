from machine import *
import os as os
import sdcard as sdcard
import time as time
import bluetooth
from micropython import const

uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1)) # Initialize UART0 pins for Ultrasonic Sensor
i2c0 = I2C(0, sda=Pin(8), scl=Pin(9), freq=400000) # Initialize I2C pins for Pressure Sensor
spi0 = SPI(0, baudrate=1000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(19), miso=Pin(16)) # Initialize SPI pins for SD Card Reader
cs0 = Pin(17, Pin.OUT, value=1) # Initialize cs pin for SD Card Reader
adc1 = ADC(27)
pwm0 = PWM(Pin(14)) # PWM pin for peristaltic pump white (speed control) wire
pwm0.freq(20000) # Keep 20kHz frequency for speed control, change duty cycle rate later
pwm1 = PWM(Pin(15)) # PWM pin for peristaltic pump green (direction) w
sd = sdcard.SDCard(spi0, cs0)

'''
IMPORTANT INFO:
gatts_write allows to write data but not send it immediately
gatts_notify allows for immediate data send
'''

## SD CARD VERIFICATION (DELETE LATER) 
os.mount(sd, "/sd")
print("Mounted! Files:",os.listdir("/sd"))
with open("/sd/log.txt", "r") as f:
        for line in f:
            print(line.strip())



## BLUETOOTH MODULE
ble = bluetooth.BLE() # create ble variable
ble.active(True) # active bluetooth
time.sleep_ms(100)
SERVICE_UUID = bluetooth.UUID("ABCDABCD-1234-ABCD-BBBB-123412341234") #defines category
CHAR_UUID    = bluetooth.UUID("ABCDABCD-1234-ABDD-CCCC-123412341235") #defines data

FLAG_NOTIFY = const(0x0010) #In use
FLAG_READ   = const(0x0002)

#Characteristics
((char_handle,),) = ble.gatts_register_services((
    (SERVICE_UUID, ((CHAR_UUID, FLAG_READ | FLAG_NOTIFY),)),
))

## Builds advertising packet
def adv_payload(name, service_uuid):
    payload = bytearray()
    payload += bytes((2, 0x01, 0x06))  # flags
    name_bytes = name.encode()
    payload += bytes((len(name_bytes) + 1, 0x09)) + name_bytes
    uuid_bytes = bytes(reversed(bytes(service_uuid)))
    payload += bytes((len(uuid_bytes) + 1, 0x07)) + uuid_bytes
    return payload


ble.config(gap_name="Oceanus") # sets name to Oceanus
ble.gap_advertise(100000, adv_payload("Oceanus", SERVICE_UUID))
print("Advertise start:")
conn_handle = None
send_pending = False

## Open log.txt and pushes to computer with gatts.notify
def send_log_file(conn_handle, path="/sd/log.txt", chunk_size=20):
    with open(path, "rb") as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            ble.gatts_notify(conn_handle, char_handle, chunk) # type: ignore
            time.sleep_ms(20)  # give the BLE stack time to flush
    ble.gatts_notify(conn_handle, char_handle, b"EOF")  #  type: ignore


## BLE events
def irq(event, data):
    global conn_handle, send_pending
    try:
        if event == 1:
            conn_handle = data[0]
            print("Laptop connected!")
            #send_log_file(conn_handle)
            send_pending = True
        elif event == 2:
            conn_handle = None
            print("Laptop disconnected")
            ble.gap_advertise(100000)
    except Exception as e:
        print("IRQ error:", e)

ble.irq(irq)

try:
    while True:
        if send_pending and conn_handle is not None:
            send_pending = False
            send_log_file(conn_handle)
        time.sleep_ms(100)
except KeyboardInterrupt:
    os.umount("/sd")
    print("SD unmounted safely")
