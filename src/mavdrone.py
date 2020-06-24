from pymavlink import mavutil
from pymavlink.mavutil import mavlink
from utils import haversineLatLonDeg
from functools import wraps
import time

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
    # Naming Convention:
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
    '''Commom Frame Arguments:
    mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT'''

    __supported_commands = [
        'waypoint', 'takeoff', 'land'
    ]

    def __init__(self, frame):
        self.frame = frame
        self.cmds = [Command.takeoff(0, 0, 1)] # pymavlink bug. First item always corrupted.
        for name in self.__supported_commands:
            setattr(self, name, self.__decorator(getattr(self, name)))

    def __decorator(self, func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            self.cmds.append(func(*args, **kwargs))
            return self
        return wrapper

    def localize(self, yx=None, latlon=None):
        for cmd in self.cmds:
            cmd.localize(yx=yx, latlon=latlon)
        return self

    def from_waypoints(self, waypoints, height=None, takeoff=False, land=False, accept_radius=0.1):
        assert(len(waypoints) > 1)
        xyz = lambda waypoint: (waypoint[0], waypoint[1], waypoint[2] if height is None else height)
        if takeoff: 
            self.takeoff(*xyz(waypoints[0]))
        for waypoint in waypoints[1 if takeoff else None: -1 if land else None]:
            self.waypoint(*xyz(waypoint), accept_radius=accept_radius)
        if land:
            self.land(*xyz(waypoints[-1]))
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

    def alt(self, rel=True):
        msg = self.get_msg('GLOBAL_POSITION_INT')
        return (msg.relative_alt if rel else msg.alt) / 1e3

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

    def upload_mission(self, mission):
        self.wait_heartbeat()
        print(f'Sending Mission Items: {len(mission.cmds)}')
        self.conn.waypoint_count_send(len(mission.cmds))
        while True:
            msg = self.get_msg('MISSION_REQUEST', 'MISSION_ACK')
            if msg.get_type() == 'MISSION_REQUEST':
                print(msg.seq, end=' ')
                cmd = mission.cmds[msg.seq]
                self.mav.mission_item_send(
                    self.conn.target_system, self.conn.target_component,
                    msg.seq, mission.frame, cmd.id, 0, 0, *cmd.params)
            elif msg.get_type() == 'MISSION_ACK':
                print('\n', 'MAV_MISSION_RESULT', msg.type)
                return msg

    # Non-pymavlink (No need heartbeat)
    def start_mission(self):
        self.set_mode(Mode.GUIDED)
        self.arm()
        self.send_command(Command.takeoff(0, 0, 1))
        while self.alt() < 0.5: pass
        self.set_mode(Mode.AUTO)

    def wait_for_mission_item(self, item):
        '''Mission item starts from 1'''
        while self.get_msg('MISSION_ITEM_REACHED').seq != item:
            pass