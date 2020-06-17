from gazebo import gztopic
from mavdrone import mavlink, Mode, Mission, Drone
from proto.diagnostics_pb2 import Diagnostics
import time

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

def get_gz_time():
    msg = gztopic('/gazebo/default/diagnostics', Diagnostics)
    real_time = msg.real_time.sec + msg.real_time.nsec / 1e9
    sim_time = msg.sim_time.sec + msg.sim_time.nsec / 1e9
    return (real_time, sim_time)

if __name__ == "__main__":
    drone = Drone()

    upload_square_mission(drone, 10, 30)

    drone.start_mission()

    drone.wait_for_mission_item(2)

    real_start, sim_start = get_gz_time()
    start = time.time()
    print('Timer Starts', start)

    drone.wait_for_mission_item(7)

    real_end, sim_end = get_gz_time()
    end = time.time()
    print('Timer Ends', end)

    print('Real Time Duration:', real_end - real_start)
    print('Sim Time Duration:', sim_end - sim_start)
    print('Python Time Duration:', end - start)