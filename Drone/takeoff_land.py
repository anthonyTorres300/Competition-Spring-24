from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import asyncio

from waypoint import Waypoint
from geofence import GeoFence

class Drone :
    # Initialize the drone's MAVSDK system and waypoint list
    def __init__(self, system_address="serial:///dev/ttyAMA0:57600"): 
        self.system_address = system_address
        self.drone = System()

        self.waypoint_list = [] 
        # self.geofence = GeoFence(18.2074, 18.2080, -67.1412, -67.1408, 10)
    
    async def connect(self): # Connect to the drone via MAVLink
        await self.drone.connect(self.system_address)
        print("Waiting for drone to connect...")
        # async for state in self.drone.core.connection_state():
        #     if state.is_connected:
        #         print(f"-- Connected to drone!")

    async def arm(self): # Arm the drone so it's ready for flight
        await self.connect() # Ensure that it's connected first
        res = await self.drone.action.arm() # Arm the motors
        print("Drone armed")
        # print(res.result)
    
    async def disarm(self): # Disarm the drone (stop the motors)
        await self.connect()
        await self.drone.action.disarm()
        print("Drone disarmed")

    async def takeoff(self):  # Take off and reach a preset altitude
        await self.drone.action.set_takeoff_altitude(10.0)  # Start at 10 meters
        print("Taking off")
        await self.drone.action.takeoff()
        await asyncio.sleep(10)  # Wait to stabilize in the air
        
    # async def execute(self): # The execution of the flight plan by visiting the waypoints one by one
    #     #async 
    #     while (self.waypoint_list): # Loop while there are still waypoints in the list
    #         wp = self.waypoint_list.pop() # Get the next waypoint
    #         if not self.geofence.is_within_bounds(wp):  # This checks if the waypoint is out of bounds
    #             print("The Waypoint is outside of the geofence!")
    #             continue
    #         await self.drone.action.goto_location(wp.lat, wp.lon, wp.alt, 0)
    #         print(f"Going to {wp.lat}, {wp.lon}, {wp.alt}")
    #         await asyncio.sleep(10) 
    #     await self.drone.action.land() #Land the drone after completing the waypoints
    #     print("Landing...")    

   # Execute a flight plan by visiting waypoints one by one
    async def execute(self):
        while self.waypoint_list:
            wp = self.waypoint_list.pop()
            alt = await self.drone.action.get_takeoff_altitude()
            await self.drone.action.goto_location(wp.lat, wp.lon, wp.alt, 0)
            print(f"Going to {wp.lat}, {wp.lon}, {alt}")
            await self.drone.action.hold()
            await asyncio.sleep(10)
        await self.drone.action.land()
        print("Landing...")

    # await self.drone.action.goto_location(waypoint.waypoint.lat,waypoint.waypoint.lon,waypoint.waypoint.alt)
    async def print_altitude(self):
        print(f"Print altitude recieved")
        await self.connect()
        async for position in self.drone.telemetry.position():
            altitude = round(position.relative_altitude_m)
            print(f"Altitude: {altitude}")
            asyncio.sleep(1)
        print("Done")

async def main():
    from waypoint import Waypoint

    drone = Drone()

    # Central point
    lat_base = 18.209722
    lon_base = -67.139444

    wp1 = Waypoint(lat_base, lon_base, 10)
    wp2 = Waypoint(lat_base + 0.000015, lon_base, 10)
    wp3 = Waypoint(lat_base, lon_base + 0.000015, 10)
    wp4 = Waypoint(lat_base - 0.000015, lon_base - 0.000015, 10)

    drone.waypoint_list = [wp4, wp3, wp2, wp1]  # Reverse order if using pop()

    await drone.connect()
    await drone.arm()
    await drone.takeoff()
    await drone.execute()
    await drone.disarm()

if __name__ == "__main__":
    asyncio.run(main())