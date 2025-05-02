from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import asyncio
import waypoint # Importing the waypoint class 

class Drone :
    def __init__(self, system_address="serial:///dev/ttyAMA10:57600"):
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

    # Real-world GPS coordinates converted from DMS (Degrees, Minutes, Seconds)
    wp1 = Waypoint(18.207778, -67.141111, 5)
    wp2 = Waypoint(18.207778, -67.141111, 5)
    wp3 = Waypoint(18.207778, -67.140833, 5)

    # Waypoint list in reverse order since we're using .pop()
    drone.waypoint_list = [wp1, wp3, wp2, wp1]  # Return to wp1 at the end

    await drone.connect()
    await drone.arm()
    await drone.takeoff()
    await drone.execute()
    await drone.disarm()    

if __name__ == "__main__":
    asyncio.run(main())