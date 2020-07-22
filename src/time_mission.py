from gazebo import gztopic
from mavdrone import mavlink, Mode, Mission, Drone, Command
from proto.diagnostics_pb2 import Diagnostics
import pickle
import time

def get_gz_time():
    msg = gztopic('/gazebo/default/diagnostics', Diagnostics)
    real_time = msg.real_time.sec + msg.real_time.nsec / 1e9
    sim_time = msg.sim_time.sec + msg.sim_time.nsec / 1e9
    return (real_time, sim_time)

def upload_square_mission(drone, h, r):
    drone.upload_mission(
        Mission(mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
            .takeoff(0, 0, h)
            .waypoint(r, 0, h)
            .waypoint(r, r, h)
            .waypoint(-r, r, h)
            .waypoint(-r, -r, h)
            .waypoint(r, -r, h)
            .waypoint(r, 0, h)
            .land(0, 0, 0)
            .localize(latlon=drone.latlon()))

def read_waypoints(filename):
    with open(filename, 'rb') as file:
        waypoints = pickle.load(file)
        return waypoints

def time_mission(drone, start_item, end_item):
    if start_item != 0:
        drone.wait_for_mission_item(start_item)
    real_start, sim_start = get_gz_time()
    start = time.time()
    print('Timer Starts')

    drone.wait_for_mission_item(end_item)
    real_end, sim_end = get_gz_time()
    end = time.time()
    print('Timer Ends')

    real_duration = real_end - real_start
    sim_duration = sim_end - sim_start
    duration = end - start
    print('Real Time Duration:', real_duration)
    print('Sim Time Duration:', sim_duration)
    print('Python Time Duration:', duration)

    return real_duration, sim_duration, duration

def upload_and_time_mission(drone, waypoints, height=10, land=False):
    drone.upload_mission(
        Mission(mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
            .from_waypoints(waypoints, height=height, land=land)
            .localize(latlon=drone.latlon()))

    drone.start_mission()
    time_mission(drone, 0, len(waypoints))

if __name__ == "__main__":
    # Create a Drone Object (It will connect to ArduPilot)
    drone = Drone()

    ### Example 1: Upload and Time a Square Mission

    upload_square_mission(drone, 15, 20)

    drone.start_mission()

    drone.wait_for_mission_item(2)

    real_start, sim_start = get_gz_time()

    drone.wait_for_mission_item(7)

    real_end, sim_end = get_gz_time()

    print('Real Time Duration:', real_end - real_start)
    print('Sim Time Duration:', sim_end - sim_start)

    ### Example 2: Read, Upload & Time Waypoints

    # spiral_waypoints = read_waypoints('src/spiral')
    # print('Spiral Waypoints:\n', spiral_waypoints)
    # upload_and_time_mission(drone, spiral_waypoints, height=50, land=True)
    
    # ladder_waypoints = read_waypoints('src/ladder')
    # print('Ladder Waypoints\n', ladder_waypoints)
    # upload_and_time_mission(drone, ladder_waypoints, height=50, land=True)