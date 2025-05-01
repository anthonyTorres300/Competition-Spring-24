import time
import rc

# Connect to the MAVProxy instance running on the rover's autopilot
rc = rc.RC()
print("Connected")


print(rc.arm())
print("Arm succesfully")

# if not rc.check_gps_status():
#     print("Exiting: Error - No GPS fix.")
#     exit()

# Current location and new location
pos = rc.get_position()
lat = pos[0]
lon = pos[1]
print("Current Latitude:", lat, "Current Longitude:", lon)

time.sleep(5)
# Move to desired lat and lon
new_lat = lat + 1.0
new_lon = lon + 1.0
rc.send_waypoint(new_lat, new_lon)
# print("Move from: ", lat, lon, "to: ", new_pos[0], new_pos[1])

waypoints = [
    (18.4655, -66.1057, 0.0),  # San Juan, PR
    (18.4700, -66.1100, 0.0),  # Second point
    (18.4750, -66.1150, 0.0),  # Third point
]

rc.mission_mode(waypoints)

# Run forward for 5 seconds
time.sleep(10)

# # Disarm the rover
rc.disarm()
print("Mission completed. Rover disarmed.")
