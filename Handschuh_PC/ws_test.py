import asyncio
import websockets

async def handler(ws):
    print("Client connected")
    await ws.wait_closed()

async def main():
    async with websockets.serve(handler, "localhost", 8080):
        print("Server läuft")
        await asyncio.Future()

asyncio.run(main())