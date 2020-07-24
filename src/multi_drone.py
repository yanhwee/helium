from mavdrone import mavlink, Drone, Command, Mission, Mode

def create_square_mission(h, r):
    return (
        Mission(mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)
            .takeoff(0, 0, h)
            .waypoint(r, 0, h)
            .waypoint(r, r, h)
            .waypoint(-r, r, h)
            .waypoint(-r, -r, h)
            .waypoint(r, -r, h)
            .waypoint(r, 0, h)
            .land(0, 0, 0))

if __name__ == "__main__":
    drone1 = Drone('udpin:127.0.0.1:14550')
    drone2 = Drone('udpin:127.0.0.1:14560')

    mission = create_square_mission(15, 120)
    mission.localize(latlon=drone1.latlon())

    drone1.upload_mission(mission)
    drone2.upload_mission(mission)

    drone1.start_mission()
    drone2.start_mission()