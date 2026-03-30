import asyncio
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "JDY-23"
NOTIFY_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

def handle_data(sender, data):
    raw = bytes(data)
    print(f"[raw] {raw}")

async def main():
    print(f"Scanning for {DEVICE_NAME}...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    if device is None:
        print("Device not found.")
        return

    print(f"Found {device.name} at {device.address}, connecting...")
    async with BleakClient(device) as client:
        # Find notify characteristic
        notify_uuid = None
        for service in client.services:
            for char in service.characteristics:
                if "notify" in char.properties:
                    notify_uuid = char.uuid
                    break
            if notify_uuid:
                break

        if notify_uuid is None:
            notify_uuid = NOTIFY_UUID  # fallback to known UUID

        await client.start_notify(notify_uuid, handle_data)
        print(f"Connected. Listening on {notify_uuid} (Ctrl+C to stop)...\n")
        try:
            while True:
                await asyncio.sleep(0.1)
        except KeyboardInterrupt:
            pass
        await client.stop_notify(notify_uuid)
    print("Disconnected.")

if __name__ == "__main__":
    asyncio.run(main())
