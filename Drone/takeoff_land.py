from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import asyncio
import waypoint # Importing the waypoint class 

class Drone :
    def __init__(self, system_address="serial:///dev/ttyAMA0:921600"):
        # Initialize the drone's MAVSDK system and waypoint list
        self.system_address = system_address
        self.drone = System()
        self.waypoint_list = []
    
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

    async def takeoff(self): # Take off and reach a preset altitude (in this case it's 2 meters)
        await self.drone.action.set_takeoff_altitude(2.0) #Sets the desired takeoff altitude in meters (can be tweaked later)
        print("Taking off")
        await self.drone.action.takeoff()
        await asyncio.sleep(10) # Wait for the drone to stabilize in the air
        
    async def execute(self): # The execution of the flight plan by visiting the waypoints one by one
        #async 
        while (self.waypoint_list): # Loop while there are still waypoints in the list
            wp = self.waypoint_list.pop() # Get the next waypoint
            await self.drone.action.goto_location(  # Go to specific GPS location (latitude, longitude, altitude)
                wp.lat, wp.lon, 
                self.drone.action.get_takeoff_altitude(),0)
            await self.drone.action.hold() # Pause at the waypoint when reaches
            await asyncio.sleep(10) 
        await self.drone.action.land() #Land the drone after completing the waypoints
            
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
    from waypoint import Waypoint

    drone = Drone()

    # Central point
    lat_base = 18.209722
    lon_base = -67.139444
 
    # Approx. 2 meters apart
    wp1 = Waypoint(lat_base, lon_base, 10)                         # Original point
    wp2 = Waypoint(lat_base + 0.000015, lon_base, 10)              # ≈ 1.6m north
    wp3 = Waypoint(lat_base, lon_base + 0.000015, 10)              # ≈ 1.4m east
    wp4 = Waypoint(lat_base - 0.000015, lon_base - 0.000015, 10)   # ≈ 2m southwest

    drone.waypoint_list = [wp1, wp4, wp3, wp2, wp1]  # Reverse order if using pop()

    await drone.connect()
    await drone.arm()
    await drone.takeoff()
    await drone.execute()
    await drone.disarm()    

if __name__ == "__main__":
    asyncio.run(main())