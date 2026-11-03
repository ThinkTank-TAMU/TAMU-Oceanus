"""
Mission-station receiver for the float (runs on the laptop, Python 3 + bleak).

    python BLE_Reciever.py           connect, sync the clock, show live packets
    python BLE_Reciever.py --go      ... then tell the float to descend
    python BLE_Reciever.py --dump    ... then download the stored log

On connect the float's clock is set to UTC, so packets read
[EX01 1:51:42 UTC 9.8 kpa 1.00 meters]. Live packets are appended to
live_packets.txt; a log download is saved to received_log.txt and
received_log.csv (seconds, kPa, depth) for the graph.
"""

import argparse
import asyncio
import csv
import time

from bleak import BleakClient, BleakScanner

import float_logic

SERVICE_UUID = "ABCDABCD-1234-ABCD-BBBB-123412341234"
TX_UUID = "ABCDABCD-1234-ABDD-CCCC-123412341235"   # float -> station (notify)
RX_UUID = "ABCDABCD-1234-ABDD-CCCC-123412341236"   # station -> float (write)


class Receiver:
    def __init__(self):
        self.buf = ""
        self.log = None             # list while a log download is in progress
        self.done = asyncio.Event()

    def on_notify(self, _sender, data):
        self.buf += data.decode("utf-8", errors="replace")
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            self.on_line(line.strip())

    def on_line(self, line):
        if line == "BEGIN LOG":
            self.log = []
            print("Receiving log...")
        elif line == "EOF" and self.log is not None:
            self.save(self.log)
            self.log = None
            self.done.set()
        elif self.log is not None:
            self.log.append(line)
        else:
            print(line)
            if float_logic.parse_packet(line):
                with open("live_packets.txt", "a") as f:
                    f.write(line + "\n")

    @staticmethod
    def save(lines):
        with open("received_log.txt", "w") as f:
            f.write("\n".join(lines) + "\n")
        rows = [p for p in map(float_logic.parse_packet, lines) if p]
        with open("received_log.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["seconds", "clock", "kpa", "depth_m"])
            for p in rows:
                w.writerow([p["seconds"], p["clock"], p["kpa"], p["depth_m"]])
        print("Saved {} lines ({} packets) to received_log.txt / .csv".format(len(lines), len(rows)))


async def main(args):
    print("Searching for the float...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: SERVICE_UUID.lower() in [s.lower() for s in ad.service_uuids],
        timeout=args.timeout)
    if not device:
        print("Float not found!")
        return

    rx = Receiver()
    print("Found float, connecting to {}...".format(device.address))
    async with BleakClient(device) as client:
        print("Connected.")
        await client.start_notify(TX_UUID, rx.on_notify)
        await client.write_gatt_char(RX_UUID, "T{}".format(int(time.time())).encode(), response=True)
        if args.go:
            await asyncio.sleep(2.0)    # the float must have sent a packet first
            await client.write_gatt_char(RX_UUID, b"GO", response=True)
        if args.dump:
            await client.write_gatt_char(RX_UUID, b"DUMP", response=True)
        print("Listening... (Ctrl+C to stop)")
        while client.is_connected:
            await asyncio.sleep(1)
        print("Disconnected.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--go", action="store_true", help="command the descent after the first packet")
    ap.add_argument("--dump", action="store_true", help="download the stored packet log")
    ap.add_argument("--timeout", type=float, default=10.0, help="scan timeout, s")
    try:
        asyncio.run(main(ap.parse_args()))
    except KeyboardInterrupt:
        pass
