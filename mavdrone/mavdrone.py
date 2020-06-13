from pymavlink import mavutil
from pymavlink.mavutil import mavlink
from utils import haversineLatLonDeg
from functools import wraps

class Mode:
    STABILIZE = 0;      ACRO = 1;       ALT_HOLD = 2;   AUTO = 3;   GUIDED = 4
    LOITER = 5;         RTL = 6;        CIRCLE = 7;     LAND = 9;   DRIFT = 11
    SPORT = 13;         POSHOLD = 16;   BRAKE = 17;     THROW = 18; AVOID_ADSB = 19
    GUIDED_NOGPS = 20;  SMART_RTL = 21; ZIGZAG = 24

class Command:
    def __init__(self, id, *params):
        self.id = id
        self.params = list(params)
    
    # Format:
    # - First 3 params: lat=0, lon=0, alt=0
    # - Remaining params: Message field name order
    # Naming Substitution:
    # - Latitude: lat
    # - Longtitude: lon
    # - Altitude: alt
    # - Yaw Angle: yaw
    # - Not Stated: Name in snake_case

    @staticmethod
    def waypoint(lat=0, lon=0, alt=0, hold=0, accept_radius=0.1, pass_radius=0, yaw=0):
        return Command(mavlink.MAV_CMD_NAV_WAYPOINT,
        hold, accept_radius, pass_radius, yaw, lat, lon, alt)

    @staticmethod
    def takeoff(lat=0, lon=0, alt=0, pitch=0, yaw=0):
        return Command(mavlink.MAV_CMD_NAV_TAKEOFF,
        pitch, 0, 0, yaw, lat, lon, alt)

    @staticmethod
    def land(lat=0, lon=0, alt=0, abort_alt=0, land_mode=mavlink.PRECISION_LAND_MODE_DISABLED, yaw=0):
        return Command(mavlink.MAV_CMD_NAV_LAND,
        abort_alt, land_mode, 0, yaw, lat, lon, alt)

    # Other Methods
    def localize(self, yx=None, latlon=None):
        assert(bool(yx) is not bool(latlon))
        if yx:
            self.params[-3] += yx[0]
            self.params[-2] += yx[1]
        elif latlon:
            self.params[-3:-1] = haversineLatLonDeg(*latlon, self.params[-3], self.params[-2])

class Mission(Command):
    def __init__(self):
        self.cmds = []
        for name in ['waypoint', 'takeoff', 'land']:
            setattr(self, name, self.__decorator(getattr(self, name)))

    def __decorator(self, func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            self.cmds.append(func(*args, **kwargs))
            return self
        return wrapper

    def __getitem__(self, key):
        return self.cmds[key]

    def __len__(self):
        return len(self.cmds)

    def localize(self, yx=None, latlon=None):
        for cmd in self.cmds:
            cmd.localize(yx=yx, latlon=latlon)
        return self

class Drone:
    def __init__(self, conn='udpin:127.0.0.1:14550'):
        conn = mavutil.mavlink_connection(conn)
        conn.wait_heartbeat()

        self.conn = conn
        self.mav = conn.mav
        self.waypoints = []

    # def heartbeat(func):
    #     @wraps(func)
    #     def wrapper(self, *args, **kwargs):
    #         self.conn.wait_heartbeat()
    #         func()
    #     return wrapper

    def wait_heartbeat(self):
        self.conn.wait_heartbeat()
    
    def get_msg(self, *types):
        if len(types) == 0: return
        return self.conn.recv_match(type=list(types), blocking=True)
    
    def latlon(self):
        msg = self.get_msg('GLOBAL_POSITION_INT')
        return msg.lat / 1e7, msg.lon / 1e7

    def arm(self):
        self.wait_heartbeat()
        self.conn.arducopter_arm()
        return self.get_msg('COMMAND_ACK')

    def disarm(self):
        self.wait_heartbeat()
        self.conn.arducopter_disarm()
        return self.get_msg('COMMAND_ACK')

    def get_mode(self):
        self.conn.wait_heartbeat()
        return self.conn.flightmode

    def set_mode(self, mode):
        self.wait_heartbeat()
        self.mav.set_mode_send(self.conn.target_system, 1, mode)
        return self.get_msg('COMMAND_ACK')

    def get_param(self, name):
        self.wait_heartbeat()
        self.conn.param_fetch_one(name)
        return self.get_msg('PARAM_VALUE')

    def set_param(self, name, value, param_type=None):
        self.wait_heartbeat()
        self.conn.param_set_send(name, value, parm_type=param_type)
        return self.get_msg('PARAM_VALUE')

    def send_command(self, cmd):
        self.wait_heartbeat()
        self.mav.command_long_send(
            self.conn.target_system, self.conn.target_component, 
            cmd.id, 0, *cmd.params)
        return self.get_msg('COMMAND_ACK')

    def upload_mission(self, cmds, frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT):
        self.wait_heartbeat()
        print(f'Sending Mission Items: {len(cmds)}')
        self.conn.waypoint_count_send(len(cmds))
        while True:
            msg = self.get_msg('MISSION_REQUEST', 'MISSION_ACK')
            if msg.get_type() == 'MISSION_REQUEST':
                print(msg.seq, end=' ')
                cmd = cmds[msg.seq]
                params = cmd.params
                self.mav.mission_item_send(
                    self.conn.target_system, self.conn.target_component,
                    msg.seq, frame, cmd.id, 0, 0, *params)
            elif msg.get_type() == 'MISSION_ACK':
                print('\n', 'MAV_MISSION_RESULT', msg.type)
                break

if __name__ == "__main__":
    from pprint import pprint
    from itertools import product
    import pickle
    import time
    drone = Drone()
    conn, mav = drone.conn, drone.mav

    h = 10

    drone.upload_mission(
        Mission()
            .takeoff(0, 0, h)
            .waypoint(120, 0, h)
            .waypoint(120, 120, h)
            .waypoint(-120, 120, h)
            .waypoint(-120, -120, h)
            .waypoint(120, -120, h)
            .waypoint(120, 0, h)
            .land(0, 0, 0)
            .localize(latlon=drone.latlon()),
        frame=mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)

    # drone.upload_mission(
    #     Mission()
    #         .takeoff(0, 0, h)
    #         .waypoint(30, 0, h)
    #         .waypoint(30, 30, h)
    #         .waypoint(-30, 30, h)
    #         .waypoint(-30, -30, h)
    #         .waypoint(30, -30, h)
    #         .waypoint(30, 0, h)
    #         .land(0, 0, 0)
    #         .localize(latlon=drone.latlon()),
    #     frame=mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)

    # a = (Mission()
    #     .takeoff(0, 0, 10)
    #     .waypoint(120, 0, 10)
    #     .waypoint(120, 120, 10)
    #     .waypoint(-120, 120, 10)
    #     .waypoint(-120, -120, 10)
    #     .waypoint(120, -120, 10)
    #     .waypoint(120, 0, 120)
    #     .land(0, 0, 0)
    #     .localize(yx=(1,1)))

    # print(a)

    # drone.set_param('WPNAV_SPEED', 100e3)
    # drone.upload_mission(CMD.localize([
    #     CMD.takeoff(0, 0, 10),
    #     CMD.waypoint(30, 0, 10),
    #     CMD.waypoint(30, 30, 10),
    #     CMD.waypoint(-30, 30, 10),
    #     CMD.waypoint(-30, -30, 10),
    #     CMD.waypoint(30, -30, 10),
    #     CMD.waypoint(30, 0, 10),
    #     CMD.land(0, 0, 0)
    # ], latlon=drone.latlon()), frame=mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT)

    # lat, lon = drone.latlon()
    # print(lat, lon)
    # mav.terrain_check_encode(lat, lon)
    # msg = drone.get_msg('TERRAIN_REPORT')
    # print(msg)

    # mav.command_long_send(
    #     conn.target_system, conn.target_component,
    #     mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
    #     mavlink.MAVLINK_MSG_ID_ALTITUDE, 0,
    #     0, 0, 0, 0, 0
    # )

    # msg = drone.get_msg('ALTITUDE')
    # print(msg)

    # time.sleep(2)
    # drone.set_mode(Mode.GUIDED)
    # time.sleep(2)
    # drone.arm()
    # time.sleep(2)
    # drone.send_command(CMD.takeoff(0, 0, 1))
    # time.sleep(2)
    # drone.set_mode(Mode.AUTO)
    
    # print(drone.set_mode(Mode.GUIDED))
    
    # print(drone.arm())
    
    # print(drone.send_command(CMD.takeoff(alt=1)))
    
    # print(drone.set_mode(Mode.AUTO))

    # print('Mode 1')
    # drone.set_mode(Mode.ALT_HOLD)
    # print('Arm')
    # drone.arm()
    # print('AUTO')
    # drone.set_mode(Mode.AUTO)