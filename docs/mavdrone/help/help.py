from pymavlink import mavutil

drone = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
drone.wait_heartbeat()

help(drone.mav)