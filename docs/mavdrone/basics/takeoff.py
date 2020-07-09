from pymavlink import mavutil

drone = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
drone.wait_heartbeat()

drone.arducopter_arm()
drone.mav.command_long_send(
    drone.target_system, drone.target_component, 
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 
    0, 0, 0, 0, 
    0, 0, 5                                         # Lat, Long, Alt
)