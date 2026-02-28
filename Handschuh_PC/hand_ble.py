import asyncio
from time import time
from bleak import BleakScanner, BleakClient
import struct

# ==============================
# Parser-Klasse
# ==============================
class HandDataParser:
    """
    Parst ein Paket vom STM32 Handschuh.
    """

    EXPECTED_LENGTH = 86  # Byte

    @staticmethod
    def parse(data: bytes) -> dict:
        if len(data) < HandDataParser.EXPECTED_LENGTH:
            raise ValueError(f"Unerwartete Paketgröße: {len(data)} Byte, erwartet {HandDataParser.EXPECTED_LENGTH}")

        idx = 0
        # --- ADC Werte (uint16, Big-Endian) ---
        adc0 = (data[idx] << 8) | data[idx + 1]; idx += 2
        adc1 = (data[idx] << 8) | data[idx + 1]; idx += 2
        adc2 = (data[idx] << 8) | data[idx + 1]; idx += 2
        adc3 = (data[idx] << 8) | data[idx + 1]; idx += 2
        adc4 = (data[idx] << 8) | data[idx + 1]; idx += 2

        # --- 18 Float-Werte (little endian) ---
        floats = []
        for _ in range(18):
            f_bytes = data[idx:idx+4]
            val = struct.unpack('<f', f_bytes)[0]
            floats.append(val)
            idx += 4

        return {
            "adc0": adc0,
            "adc1": adc1,
            "adc2": adc2,
            "adc3": adc3,
            "adc4": adc4,
            "euler": floats[0:3],
            "gravity": floats[3:6],
            "gyro": floats[6:9],
            "accel": floats[9:12],
            "mag": floats[12:15],
            "linAccel": floats[15:18],
        }


# ==============================
# BLE Receiver-Klasse
# ==============================
class HandBLEReceiver:
    """
    Scannt nach STM32 BLE-Gerät, verbindet und liefert Sensor-Updates via Callback.
    """

    def __init__(self, name=None, address=None):
        self.name = name
        self.address = address
        self.char_uuid = None
        self.client = None
        self._last_time = time()
        self._counter = 0

    async def find_device(self):
        print("Scanne nach BLE-Geräten…")
        devices = await BleakScanner.discover()
        for d in devices:
            print(f"Device: {d.address}: {d.name}")
            if self.address and d.address.lower() == self.address.lower():
                print(f"Gefunden: {d.name} ({d.address})")
                self.address = d.address
                return d.address
            if self.name and d.name == self.name:
                print(f"Gefunden: {d.name} ({d.address})")
                self.address = d.address
                return d.address
        print("Gerät nicht gefunden!")
        return None

    async def get_notify_char(self):
        """Sucht die Notify-Characteristic im aktuellen Client"""
        if self.client is None or not self.client.is_connected:
            raise RuntimeError("Client nicht verbunden")

        for service in self.client.services:
            for char in service.characteristics:
                if "notify" in char.properties:
                    print(f"Notify-Characteristic gefunden: {char.uuid}")
                    self.char_uuid = char.uuid
                    return char.uuid
        print("Keine notify-Characteristic gefunden!")
        return None
    
    async def start(self, callback):
        """
        Startet den BLE-Empfang und ruft callback(sensor_dict) auf.
        """
        if not self.address:
            raise ValueError("BLE-Adresse muss gesetzt sein!")

        self.client = BleakClient(self.address)
        await self.client.connect()
        print(f"Verbunden: {self.client.is_connected}")

        # Characteristic nur einmal suchen
        if not self.char_uuid:
            await self.get_notify_char()
            if not self.char_uuid:
                raise RuntimeError("Keine notify-Characteristic gefunden!")

        async def _handler(sender, data):
            self._counter += 1
            now = time()
            if now - self._last_time >= 1.0:
                print(f"FPS: {self._counter}")
                self._counter = 0
                self._last_time = now
            try:
                sensors = HandDataParser.parse(data)
                # Formatiert ausgeben
                #yaw, pitch, roll = sensors["euler"]
                #print(f"Euler angles [deg]: Roll={roll:6.2f}, Pitch={pitch:6.2f}, Yaw={yaw:6.2f}")

                callback(sensors)

            except Exception as e:
                print("Fehler beim Parsen:", e)

        await self.client.start_notify(self.char_uuid, _handler)
        print("Empfange Daten… STRG+C zum Stoppen")

        # Keep alive
        while True:
            await asyncio.sleep(1)

    async def stop(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            print("BLE getrennt")