# MAIN - runs on the Pico at power-up. Combines all modules into one loop.
# Current modules: UltraSonic_Sensor (distance -> SD card), Bluetooth_Sender (SD card -> laptop)
# To add a new sensor/feature: give its file an `if __name__ == "__main__":` guard,
# import it below, and call its functions inside the main loop.
# Needs every imported file plus sdcard.py copied onto the Pico.

import os
import time
import Bluetooth_Sender as bt      # mounts SD + starts advertising as "Oceanus" on import
import UltraSonic_Sensor as us
# future modules go here, e.g.  import Depth_Sensor as depth

READ_INTERVAL_MS = 1000
last_read = time.ticks_ms() - READ_INTERVAL_MS

print("Logging to", us.LOG_FILE, "and waiting for laptop...")

try:
    while True:
        # ultrasonic: read, print, save once per interval
        if time.ticks_diff(time.ticks_ms(), last_read) >= READ_INTERVAL_MS:
            last_read = time.ticks_ms()
            dist = us.read_distance()
            if dist is None:
                print("Distance: no valid reading")
            else:
                print("Distance:", dist, "mm")
            us.save_reading(dist)

        # bluetooth: send the log when the laptop asks (or 5s after connecting)
        if bt.conn_handle is not None:
            if not bt.send_pending and bt.connect_time is not None:
                if time.ticks_diff(time.ticks_ms(), bt.connect_time) > 5000:
                    print("No GO received, sending anyway")
                    bt.send_pending = True
            if bt.send_pending:
                bt.send_pending = False
                bt.connect_time = None
                bt.send_log_file(bt.conn_handle, path=us.LOG_FILE)

        time.sleep_ms(50)
except KeyboardInterrupt:
    os.umount("/sd")
