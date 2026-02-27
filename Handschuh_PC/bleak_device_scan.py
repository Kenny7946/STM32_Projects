import asyncio
from bleak import BleakScanner, BleakClient

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
    print(f"Daten empfangen ({sender}): {list(data)}")

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