# Run this on the laptop (not the Pico):  python Download-Distance.py
# Connects to Oceanus over Bluetooth, downloads the distance log, saves it as distance.txt

from bleak import BleakClient, BleakScanner
import asyncio
import os

DEVICE_NAME = "Oceanus"
CHAR_UUID = "ABCDABCD-1234-ABDD-CCCC-123412341235"
OUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "distance.txt")
IDLE_TIMEOUT = 10  # seconds with no data before giving up

buffer = bytearray()
done = None
last_chunk = None

def on_data(sender, data):
    global last_chunk
    last_chunk = asyncio.get_running_loop().time()
    if data == b"EOF":
        done.set()
        return
    if data.startswith(b"ERR:"):
        print(f"\nPico error: {data.decode(errors='replace')}")
        return
    buffer.extend(data)
    print(f"\rReceived {len(buffer)} bytes", end="")

async def wait_for_eof():
    # keeps waiting as long as chunks keep arriving, so big logs don't time out
    loop = asyncio.get_running_loop()
    while not done.is_set():
        if loop.time() - last_chunk > IDLE_TIMEOUT:
            return False
        await asyncio.sleep(0.2)
    return True

def save_and_summarize():
    text = buffer.decode(errors="replace")
    with open(OUT_FILE, "w") as f:
        f.write(text)
    print(f"Saved to {OUT_FILE}")

    distances = []
    for line in text.splitlines():
        parts = line.strip().split(",")
        if len(parts) == 2 and parts[1].isdigit():
            distances.append(int(parts[1]))
    if distances:
        print(f"{len(distances)} readings | min {min(distances)} mm | max {max(distances)} mm")

async def run():
    global done, last_chunk
    done = asyncio.Event()

    print(f"Searching for {DEVICE_NAME}...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15)
    if not device:
        print("Device not found! Is the Pico powered and advertising?")
        return

    async with BleakClient(device) as client:
        print("Connected, requesting log...")
        await client.start_notify(CHAR_UUID, on_data)
        last_chunk = asyncio.get_running_loop().time()
        await client.write_gatt_char(CHAR_UUID, b"GO")  # tells the Pico to start sending now
        finished = await wait_for_eof()
        await client.stop_notify(CHAR_UUID)

    print()
    if not finished:
        print(f"No data for {IDLE_TIMEOUT}s, transfer incomplete.")
    if buffer:
        save_and_summarize()
    else:
        print("Nothing received.")

if __name__ == "__main__":
    asyncio.run(run())
