import asyncio
from time import time
from bleak import BleakScanner, BleakClient

last_time = time()
counter = 0

import struct

def parse_hand_data(data: bytes):
    """
    Parst ein Paket vom STM32 Handschuh.

    data: bytes, Länge = 76
    Rückgabe: dict mit allen Sensoren
    """
    if len(data) < 86:
        raise ValueError(f"Unerwartete Paketgröße: {len(data)} Byte, erwartet 86")

    idx = 0
    # --- ADC Werte (uint16, Big-Endian) ---
    adc0 = (data[idx] << 8) | data[idx + 1]
    idx += 2
    adc1 = (data[idx] << 8) | data[idx + 1]
    idx += 2
    adc2 = (data[idx] << 8) | data[idx + 1]
    idx += 2
    adc3 = (data[idx] << 8) | data[idx + 1]
    idx += 2
    adc4 = (data[idx] << 8) | data[idx + 1]
    idx += 2

    # --- 18 Float-Werte ---
    floats = []
    for _ in range(18):
        f_bytes = data[idx:idx+4]
        val = struct.unpack('<f', f_bytes)[0]  # STM32 float = little-endian
        floats.append(val)
        idx += 4

    # Mapping zu Sensoren
    result = {
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

    return result

async def find_device(name=None, address=None):
    print("Scanne nach BLE-Geräten…")
    devices = await BleakScanner.discover()
    for d in devices:
        print(f"Device: {d.address}: {d.name}")
        if address and d.address.lower() == address.lower():
            print(f"Gefunden: {d.name} ({d.address})")
            return d.address
        if name and d.name == name:
            print(f"Gefunden: {d.name} ({d.address})")
            return d.address
    print("Gerät nicht gefunden!")
    return None

async def get_notify_char(address):
    async with BleakClient(address) as client:
        print("Verbunden:", client.is_connected)
        # Services sind bereits geladen nach connect
        for service in client.services:
            for char in service.characteristics:
                if "notify" in char.properties:
                    print(f"Notify-Characteristic gefunden: {char.uuid}")
                    return char.uuid
    print("Keine notify-Characteristic gefunden!")
    return None

def handle(sender, data):
    #print(f"Daten empfangen ({sender}): {list(data)}")
    global counter, last_time
    counter += 1
    now = time()
    if now - last_time >= 1.0:
        #print(f"FPS: {counter} | Letztes Paket: {len(data)} Byte")
        counter = 0
        last_time = now

    try:
        sensors = parse_hand_data(data)
        print(f"ADC: {sensors['adc0']}, {sensors['adc1']}, {sensors['adc2']}, {sensors['adc3']}, {sensors['adc4']}")
        #print(f"Euler: {sensors['euler']}")
        #print(f"Gyro: {sensors['gyro']}")
    except Exception as e:
        print("Fehler beim Parsen:", e)

async def main():
    # Optional: MAC-Adresse deines STM32 hier eintragen
    STM32_ADDRESS = None
    STM32_NAME = "XX-STM32"

    address = await find_device(name=STM32_NAME, address=STM32_ADDRESS)
    if not address:
        return

    char_uuid = await get_notify_char(address)
    if not char_uuid:
        return

    async with BleakClient(address) as client:
        await client.start_notify(char_uuid, handle)
        print("Empfange Daten… STRG+C zum Stoppen")
        while True:
            await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())