from pymavlink import mavutil

device = "/dev/tty.usbserial-DA00CBH4"
baud = 115200

print(f"Connecting to {device} at {baud} baud...")
master = mavutil.mavlink_connection(device, baud=baud)

print("Waiting for heartbeat...")
hb = master.wait_heartbeat(timeout=5)

if hb:
    print(f"Heartbeat received from system {master.target_system}")
else:
    print("No heartbeat received. Check telemetry.")
