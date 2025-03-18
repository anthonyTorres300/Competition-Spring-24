from pymavlink import mavutil
from utilities.connect_to_sysid import connect_to_sysid
from utilities.wait_for_position_aiding import wait_until_position_aiding
from utilities.get_autopilot_info import get_autopilot_info
import time

main_mode_mapping_px4 = {
    'MANUAL': 0,
    'ALTHOLD': 1,
    'POSCTL': 2,
    'AUTO': 3,
    'ACRO': 4,
    'OFFBOARD': 5,
    'STABILIZE': 6,
    'RATTITUDE': 7,
}

sub_mode_mapping_px4 = {
    'READY': 0,
    'TAKEOFF': 1,
    'HOLD': 2,  # LOITER in MAVLink
    'MISSION': 3,
    'RETURN_TO_LAUNCH': 4,
    'LAND': 5,
    'FOLLOW': 6,
}


def change_mode(master, mode, autopilot='ardupilot', sub_mode='NONE'):

    if autopilot == 'ardupilot':
        # Check if mode is available
        if mode not in master.mode_mapping():
            print(f'Unknown mode : {mode}')
            print(f"available modes: {list(master.mode_mapping().keys())}")
            raise Exception('Unknown mode')
        
        # Get mode ID
        mode_id = master.mode_mapping()[mode]
        sub_mode = 0
    elif autopilot == 'px4':
        # Get mode ID
        mode_id = main_mode_mapping_px4[mode]
        sub_mode = sub_mode_mapping_px4[sub_mode]


    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, sub_mode, 0, 0, 0, 0)
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(ack_msg)
    return ack_msg.result

def arm(mav_connection, arm_command):
    # Wait for the first heartbeat
    # This sets the system and component ID of remote system for the link
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    # return the result of the ACK message
    return msg.result

def takeoff(mav_connection, takeoff_altitude: float, tgt_sys_id: int = 1, tgt_comp_id=1):

    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    wait_until_position_aiding(mav_connection)

    autopilot_info = get_autopilot_info(mav_connection, tgt_sys_id)

    if autopilot_info["autopilot"] == "ardupilotmega":
        print("Connected to ArduPilot autopilot")
        mode_id = mav_connection.mode_mapping()["GUIDED"]
        takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]

    elif autopilot_info["autopilot"] == "px4":
        print("Connected to PX4 autopilot")
        print(mav_connection.mode_mapping())
        mode_id = mav_connection.mode_mapping()["TAKEOFF"][1]
        print(mode_id)
        msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]

    else:
        raise ValueError("Autopilot not supported")


    # Change mode to guided (Ardupilot) or takeoff (PX4)
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

    # Arm the UAS
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK:  {arm_msg}")

    # Command Takeoff
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])

    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Takeoff ACK:  {takeoff_msg}")

    return takeoff_msg.result


def land(the_connection: mavutil.mavlink_connection, timeout: int = 10) -> int:
    """
    Sends a command for the drone to land.

    Args:
        the_connection (mavutil.mavlink_connection): The MAVLink connection to use.
        timeout (int): Time in seconds to wait for an acknowledgment.

    Returns:
        int: mavutil.mavlink.MAV_RESULT enum value.
    """

    # Send a command to land
    the_connection.mav.command_long_send(
        the_connection.target_system, 
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 
        0, 0, 0, 0, 0, 0, 0, 0
    )

    # Wait for the acknowledgment
    ack = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
    if ack is None:
        print('No acknowledgment received within the timeout period.')
        return None

    return ack.result



if __name__ == '__main__':

    the_connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)

    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()

    # Call the land function
    print(change_mode(the_connection, "STABILIZE", "px4", "READY"))
    arm(the_connection, 1)
    takeoff(the_connection, 1)
    time.sleep(3)
    print(change_mode(the_connection, "STABILIZE", "px4", "READY"))
    time.sleep(5)


    #   SHOOT
    


    time.sleep(5)
    land(the_connection)
    time.sleep(3)
    arm(the_connection, 0)

# mav_connection =  mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
#     takeoff(mav_connection, 1)