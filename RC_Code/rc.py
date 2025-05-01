from pymavlink import mavutil
import time
class RC:
    def __init__(self):
        self.mav = mavutil.mavlink_connection('COM4', baud=115200)
        print("Listening for MAVLink messages on Windows...")

        start_time = time.time()
        heartbeat_received = False

        while time.time() - start_time < 10:
            msg = self.mav.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                print(f"Received: {msg_type}")
                if msg_type == 'HEARTBEAT':
                    heartbeat_received = True
                    self.target_system = msg.get_srcSystem()
                    self.target_component = msg.get_srcComponent()
                    print(f"Heartbeat received from system {self.target_system}, component {self.target_component}")
                    break
            time.sleep(0.1)

        if not heartbeat_received:
            raise TimeoutError("Timeout: No heartbeat received from the flight controller.")

        self.mav.target_system = self.target_system
        self.mav.target_component = self.target_component

    def arm(self):
        print("Arming vehicle...")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )

    def disarm(self):
        print("Disarming vehicle...")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def get_position(self):
        self.mav.mav.request_data_stream_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1, 1)

        while True:
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                return lat, lon

def mission_mode(self, waypoints):
    self.mav.mav.mission_clear_all_send(self.mav.target_system, self.mav.target_component)
    time.sleep(1)

    print(f"Uploading {len(waypoints)} waypoint(s)...")
    self.mav.mav.mission_count_send(self.mav.target_system, self.mav.target_component, len(waypoints))

    for i, (lat, lon) in enumerate(waypoints):
        alt = 0.0  # Since this is just a RC Car altitude is not used but it's required for the mav command

        while True:
            msg = self.mav.recv_match(type='MISSION_REQUEST', blocking=True, timeout=10)
            if msg and msg.seq == i:
                break

        self.mav.mav.mission_item_send(
            self.mav.target_system,
            self.mav.target_component,
            i,  # sequence
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # still needed even if alt is ignored
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,  # current
            1,  # autocontinue
            0, 0, 0, 0,  # params 1-4 (unused here)
            lat, lon, alt  # lat, lon, alt
        )
        print(f"Sent waypoint {i}: ({lat}, {lon})")

    print("Starting mission...")
    self.mav.mav.command_long_send(
        self.mav.target_system,
        self.mav.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,  # confirmation
        0, 0, 0, 0, 0, 0  # unused params
    )

