from pymavlink import mavutil, mavwp
from utils import haversineLatLonDeg

drone = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
drone.wait_heartbeat()

# Local Waypoints
local_waypoints = [(15, 15, 20), (15, -15, 20), (-15, -15, 20), (-15, 15, 20)]

# Get Current Position
msg = drone.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
lat, lon = msg.lat / 1e7, msg.lon / 1e7
global_waypoints = [
    (*haversineLatLonDeg(lat, lon, x, y), z) for x, y, z in local_waypoints]

print(msg)

print('Global WPs:', global_waypoints)

# Prepare Waypoints
waypoint_loader = mavwp.MAVWPLoader()
for i, waypoint in enumerate(global_waypoints):
    waypoint_loader.add(mavutil.mavlink.MAVLink_mission_item_message(
        drone.target_system, drone.target_component,
        i,                                              # Sequence
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Coordinate Frame
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,           # Scheduled Action
        0,                                              # Current
        0,                                              # Autocontinue
        0,                                              # Hold
        0.1,                                              # Acceptance Radius
        0,                                              # Pass Radius
        180,                                              # Yaw
        waypoint[0], waypoint[1], waypoint[2]           # Lat, Long, Alt
    ))

# Send Waypoints
drone.waypoint_clear_all_send()

print(f'Send Count: {waypoint_loader.count()}')
drone.waypoint_count_send(waypoint_loader.count())

for i in range(waypoint_loader.count()):
    msg = drone.recv_match(type=['MISSION_REQUEST'], blocking=True)
    print(f'Sending waypoint {msg.seq}')
    drone.mav.send(waypoint_loader.wp(msg.seq))
    print(waypoint_loader.wp(msg.seq))