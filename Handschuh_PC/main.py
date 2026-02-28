import asyncio
from hand_ble import HandBLEReceiver

def handle_sensor_data(sensors):
    print(f"ADC0={sensors['adc0']}, ADC1={sensors['adc1']}, ADC2={sensors['adc2']}, ADC3={sensors['adc3']}, ADC4={sensors['adc4']}")
    # Hier z.B. PoseEstimator aufrufen

async def main():
    ble = HandBLEReceiver(name="XX-STM32")  # optional: address="AA:BB:CC:DD:EE:FF"
    await ble.find_device()
    await ble.start(handle_sensor_data)

if __name__ == "__main__":
    asyncio.run(main())