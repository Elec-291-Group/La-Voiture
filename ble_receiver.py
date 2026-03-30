import asyncio
from typing import List, Optional, Tuple

from PyQt5.QtCore import QThread, pyqtSignal

try:
    from bleak import BleakClient, BleakScanner
except Exception:
    BleakClient = None
    BleakScanner = None


DEFAULT_DEVICE_NAME = "JDY-23"
DEFAULT_WRITE_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"


async def _scan_devices_async(timeout: float = 5.0) -> List[Tuple[str, str]]:
    if BleakScanner is None:
        return []

    devices = await BleakScanner.discover(timeout=timeout)
    results = []
    for device in devices:
        name = device.name or device.address
        results.append((f"{name} ({device.address})", device.address))
    return results


def scan_ble_devices(timeout: float = 5.0) -> List[Tuple[str, str]]:
    if BleakScanner is None:
        return []

    try:
        return asyncio.run(_scan_devices_async(timeout=timeout))
    except Exception:
        return []


class BleWorker(QThread):
    data_received = pyqtSignal(dict)
    line_received = pyqtSignal(str)
    status_message = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)

    def __init__(self, device_address: Optional[str] = None, device_name: str = DEFAULT_DEVICE_NAME):
        super().__init__()
        self.device_address = device_address
        self.device_name = device_name
        self.running = False
        self.client = None
        self.loop = None
        self.write_uuid = None
        self.notify_uuid = None
        self._rx_buffer = b""

    def run(self):
        if BleakClient is None or BleakScanner is None:
            self.status_message.emit("Bleak not installed")
            self.connection_changed.emit(False)
            return

        self.running = True
        try:
            asyncio.run(self._run_async())
        except Exception as e:
            self.status_message.emit(f"Bluetooth worker failed: {e}")
            self.connection_changed.emit(False)

    async def _run_async(self):
        self.loop = asyncio.get_running_loop()
        device = await self._find_device()
        if device is None:
            self.connection_changed.emit(False)
            self.status_message.emit("Bluetooth device not found")
            return

        async with BleakClient(device) as client:
            self.client = client
            services = await self._get_services_compat(client)
            self.notify_uuid, self.write_uuid = self._resolve_characteristics(services)
            if self.notify_uuid is None:
                self.connection_changed.emit(False)
                self.status_message.emit("No notify characteristic found")
                return
            if self.write_uuid is None:
                self.connection_changed.emit(False)
                self.status_message.emit("No writable characteristic found")
                return

            await client.start_notify(self.notify_uuid, self._handle_data)
            self.connection_changed.emit(True)
            self.status_message.emit(
                f"Connected to {device.name or self.device_name} at {device.address}"
            )

            try:
                while self.running:
                    await asyncio.sleep(0.05)
            finally:
                try:
                    await client.stop_notify(self.notify_uuid)
                except Exception:
                    pass

        self.client = None
        self.connection_changed.emit(False)
        self.status_message.emit("Disconnected")

    async def _find_device(self):
        if self.device_address:
            device = await BleakScanner.find_device_by_address(self.device_address, timeout=10.0)
            if device is not None:
                return device

        return await BleakScanner.find_device_by_name(self.device_name, timeout=10.0)

    @staticmethod
    async def _get_services_compat(client):
        if hasattr(client, "get_services"):
            services = await client.get_services()
            if services is not None:
                return services

        services = getattr(client, "services", None)
        if services is not None:
            return services

        raise RuntimeError("BLE services unavailable on this Bleak version")

    @staticmethod
    def _resolve_characteristics(services):
        notify_uuid = None
        write_uuid = None

        for service in services:
            for char in service.characteristics:
                props = set(char.properties)
                if notify_uuid is None and "notify" in props:
                    notify_uuid = char.uuid
                if write_uuid is None and ("write" in props or "write-without-response" in props):
                    write_uuid = char.uuid

        if write_uuid is None:
            for service in services:
                for char in service.characteristics:
                    if char.uuid.lower() == DEFAULT_WRITE_UUID:
                        write_uuid = char.uuid
                        break
                if write_uuid is not None:
                    break

        return notify_uuid, write_uuid

    def _handle_data(self, sender, data):
        del sender
        self._rx_buffer += bytes(data)

        while b"\n" in self._rx_buffer:
            raw_line, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
            line = raw_line.decode("utf-8", errors="ignore").strip()
            if line:
                self.line_received.emit(line)

    async def _write_text_async(self, text: str):
        if self.client is None or self.write_uuid is None:
            return False

        payload = text.encode("utf-8")
        await self.client.write_gatt_char(self.write_uuid, payload)
        return True

    def send_text(self, text: str):
        if self.loop is None or self.client is None:
            return False

        try:
            future = asyncio.run_coroutine_threadsafe(self._write_text_async(text), self.loop)
            return bool(future.result(timeout=2.0))
        except Exception:
            return False

    def stop(self):
        self.running = False


async def main():
    devices = await _scan_devices_async()
    if not devices:
        print("No Bluetooth devices found.")
        return

    print("Bluetooth devices:")
    for label, _address in devices:
        print(f"  {label}")


if __name__ == "__main__":
    asyncio.run(main())
