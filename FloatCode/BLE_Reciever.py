import time
import asyncio
from bleak import BleakClient, BleakScanner

#ID
SERVICE_UUID = "ABCDABCD-1234-ABCD-BBBB-123412341234"
CHAR_UUID    = "ABCDABCD-1234-ABDD-CCCC-123412341235"

received_data = []

def notification_handler(sender, data):
    decoded = data.decode("utf-8")

    if decoded == "EOF":                        # transmission complete
        full_text = "".join(received_data)
        print("File received!")

        with open("received_log.txt", "w") as f:   # save to file on PC
            f.write(full_text)
        print("Saved to received_log.txt")
        received_data.clear()
    else:
        received_data.append(decoded)              # accumulate chunks
        print(f"Chunk received: {decoded}")

async def main():
    print("Searching for ROV...")
    
    # Find the device that has our specific Service UUID
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: SERVICE_UUID.lower() in [s.lower() for s in ad.service_uuids]
    )
    
    if not device:
        print("ROV not found!")
        return


    print(f"Found ROV! Connecting to {device.address}...")
    async with BleakClient(device) as client:  # connects here
        print("Connected!")

        await client.start_notify(CHAR_UUID, notification_handler)  # subscribe to notifications
        print("Listening for data... (Ctrl+C to stop)")

        while client.is_connected:   # keep alive until disconnected
            await asyncio.sleep(1)

        await client.stop_notify(CHAR_UUID)
        print("Disconnected.")
    
asyncio.run(main())
