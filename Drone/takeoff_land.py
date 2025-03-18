from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import asyncio
import waypoint

class Drone :
    def __init__(self, system_address="serial:///dev/ttyAMA0:921600"):
        self.system_address = system_address
        self.drone = System()
        self.waypoint_list = []
    
    async def connect(self):
        await self.drone.connect(self.system_address)
        print("Waiting for drone to connect...")
        # async for state in self.drone.core.connection_state():
        #     if state.is_connected:
        #         print(f"-- Connected to drone!")

    async def arm(self):
        await self.connect()
        res = await self.drone.action.arm()
        print("Drone armed")
        # print(res.result)
    
    async def disarm(self):
        await self.connect()
        await self.drone.action.disarm()
        print("Drone disarmed")

    async def takeoff(self):
        await self.drone.action.set_takeoff_altitude(2.0) #Sets the desired takeoff altitude in meters (can be tweaked later)
        print("Taking off")
        await self.drone.action.takeoff()
        await asyncio.sleep(10)
        
    async def execute(self):
        #async 
        while (self.waypoint_list):
            wp = self.waypoint_list.pop()
            await self.drone.action.goto_location(wp.lat, wp.lon, self.drone.action.get_takeoff_altitude(),0)
            await self.drone.action.hold()
            await asyncio.sleep(10)
        await self.drone.action.land() 
            
            
    #         await self.drone.action.goto_location(waypoint.waypoint.lat,waypoint.waypoint.lon,waypoint.waypoint.alt)
    async def print_altitude(self):
        print(f"Print altitude recieved")
        await self.connect()
        async for position in self.drone.telemetry.position():
            altitude = round(position.relative_altitude_m)
            print(f"Altitude: {altitude}")
            asyncio.sleep(1)
        print("Done")

async def main():
    drone = Drone()
    await drone.connect()
    await drone.arm()
    await drone.takeoff()
    await drone.execute()
    await drone.disarm()    

if __name__ == "__main__":
    asyncio.run(main())


    




