from gazebo import gztopic
from mavdrone import mavlink, Mode, Mission, Drone
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
    
if __name__ == "__main__":
    drone = Drone()

    waypoints = read_waypoints('src/square_local')

    drone.upload_mission(
        Mission(mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
            .from_waypoints(waypoints, height=10)
            .localize(latlon=drone.latlon()))

    drone.start_mission()

    time_mission(drone, 1, len(waypoints))