from bleak import BleakScanner, BleakClient
import asyncio
from dataclasses import dataclass
import csv
import time
import tempfile
import os

@dataclass
class Measurement:
    temp: float = 20.0
    rh: float = 50.0
    co2: int = 1000
    pm1: float = 10.0
    pm2: float = 10.0
    pm4: float = 10.0
    pm10: float = 10.0

    @classmethod
    def headers(self):
        return ["temp", "rh", "co2", "pm1", "pm2.5", "pm4", "pm10"]
    
    def as_array(self):
        return [self.temp, self.rh, self.co2, self.pm1, self.pm2, self.pm4, self.pm10]

async def run(client: BleakClient):
    if not client.is_connected:
        await client.connect()
        print("Connected!")

    path = os.path.join(tempfile.gettempdir(),  f"airq-{int(time.time())}.csv")
    print("Logging to", path)
    with open(path, mode="x") as file:
        writer = csv.writer(file)

        # Write headers
        writer.writerow(Measurement.headers())

        async def f(uuid):
            nonlocal client
            val = await client.read_gatt_char(uuid)
            return int.from_bytes(val, byteorder="little") / 100
        
        async def i(uuid):
            nonlocal client
            val = await client.read_gatt_char(uuid)
            return int.from_bytes(val, byteorder="little")
        
        while True:
            meas = Measurement()
            meas.temp = await f("2A6E")
            meas.rh = await f("2A6F")
            meas.co2 = await i("2B8C")
            meas.pm1 = await f("0e201142-fba1-43a9-b7c9-49020b000001")
            meas.pm2 = await f("0e201142-fba1-43a9-b7c9-49020b000002")
            meas.pm4 = await f("0e201142-fba1-43a9-b7c9-49020b000004")
            meas.pm10 = await f("0e201142-fba1-43a9-b7c9-49020b000010")

            writer.writerow(meas.as_array())
            file.flush()
            print(meas)

            await asyncio.sleep(15)

async def main():
    dev = await BleakScanner.find_device_by_address("E8:83:1E:E3:5A:41")
    if dev is None:
        print("Device not found")
        exit(1)

    async with BleakClient(dev) as client:
        await run(client)
        await client.disconnect()

asyncio.run(main())
