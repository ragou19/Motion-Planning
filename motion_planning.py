import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from scipy.interpolate import interp1d


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.in_mission = True
        self.check_state = {}

        # initialize waypoint list
        self.waypoints = []

        # check if waypoints exist. If so, load path and go.
        if 'old_waypoints' in globals():
            self.waypoints = old_waypoints
            self.send_waypoints()
            self.flight_state = States.PLANNING
        else:
            # initial state
            self.flight_state = States.MANUAL

        # initial state
        # self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and np.linalg.norm(self.local_velocity[0:2] < 0.25)):
                if abs(self.local_position[2]) < 0.01 or np.linalg.norm(self.local_velocity[0:2]) < 0.1:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    if len(self.waypoints) == 0:
                        self.plan_path()
                    else:
                        self.send_waypoints()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 10
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # DONE: read lat0, lon0 from colliders into floating point values
        csvfile = open('colliders.csv', newline='')
        reader = csv.reader(csvfile)
        latLonLine = next(reader)
        lat0 = np.float(latLonLine[0][5:])
        lon0 = np.float(latLonLine[1][5:])
        
        # DONE: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # DONE: retrieve current global position
        latCurr = self._latitude
        lonCurr = self._longitude
        altCurr = self._altitude
 
        # DONE: convert to current local position using global_to_local()
        local_coordinates_NED = global_to_local(np.array((lonCurr,latCurr,altCurr)), self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position, self.local_position))

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        # DONE: convert start position to current position rather than map center
        grid_start = (np.int(np.around(local_coordinates_NED[0] - north_offset)), np.int(np.around(local_coordinates_NED[1] - east_offset)))
        
        # Set goal as some arbitrary position on the grid (toggle later assignment to test with this one)
        grid_goal = (-north_offset + 10, -east_offset + 10)
        # grid_goal = (296, -204)


        # DONE: adapt to set goal as latitude / longitude position and convert
        # Determines random coordinates for latitude and longitude and converts them to grid destinations

        # Determine two random floats in the interval [0, 1)
        randomFloats = np.random.random_sample((2,))
        # Convert 1st float to a latitude range of (0, 90)
        randomLat = np.float(randomFloats[0]*90.)
        # Convert 2nd float to a longitude range of (0, 180)
        randomLon = np.float(randomFloats[1]*180.)

        # # minimum and maximum north coordinates
        north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
        north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

        # minimum and maximum east coordinates
        east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
        east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

        # use scipy to do interpolation for us and convert lat to north, lon to east

        lat_to_north = interp1d([0, 90], [1, 919])
        lon_to_east = interp1d([0, 180], [1, 919])
        randomNorth = lat_to_north(randomLat)
        randomEast = lon_to_east(randomLon)

        # use ratios of north to east to find actual boundary coordinates for random latitudes & longitudes

        if randomNorth > randomEast:
            ratio = randomEast / randomNorth
            randomNorth = 919
            randomEast = randomNorth * ratio
        else:
            ratio = randomNorth / randomEast
            randomEast = 919
            randomNorth = randomEast * ratio

        # new grid_goal as an array of ints
        grid_goal = np.array((np.int(randomNorth),np.int(randomEast)))
        # recast grid_goal as a tuple
        grid_goal = (int(grid_goal[0]), int(grid_goal[1]))


        # Run A* to find a path from start to goal
        # DONE: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)

        print('Local Start and Goal: ', grid_start, grid_goal)

        # Tests the goal for whether or not it appears on an obstacle. If it does, the goal is repositioned to bring it closer to the center of the map. The process is then repeated until a valid goal is found.
        def adjust_goal(p):
            badGoal = True
            stated = False
            p_old = p
            while badGoal:
                if grid[p[0], p[1]] == 1:
                    if not stated:
                        print("Goal is located on an obstacle. Repositioning....")
                        stated = True
                    p = np.array(p)
                    if p[0] > 460:
                        p[0] -= 1
                    else:
                        p[0] += 1
                    if p[1] > 460:
                        p[1] -= 1
                    else:
                        p[1] += 1
                    p = (int(p[0]), int(p[1]))
                else:
                    badGoal = False
                    if p != p_old:
                        print("New goal found: ", p)
            return p

        grid_goal = adjust_goal(grid_goal)

        path, _ = a_star(grid, heuristic, grid_start, grid_goal)

        # DONE: prune path to minimize number of waypoints

        # Recast point in array form
        def point(p):
            return np.array([p[0], p[1], 1.]).reshape(1, -1)
        # Check if three points are collinear.
        # NOTE: Adjust epsilon if path cuts into obstacles
        def collinearity_check(p1, p2, p3, epsilon=7):#1e-6):
            p1 = point(p1)
            p2 = point(p2)
            p3 = point(p3)
            m = np.concatenate((p1, p2, p3), 0)
            det = np.linalg.det(m)
            return abs(det) < epsilon
        # Prune a path of points by removing middle ones alongst lines
        def prune_path(path):
            pruned_path = [p for p in path]            
            idx = 0
            while idx < len(pruned_path) - 2:
                if collinearity_check(pruned_path[idx], pruned_path[idx+1], pruned_path[idx+2]):
                    pruned_path.remove(pruned_path[idx+1])
                else:
                    idx += 1
            return pruned_path

        path = prune_path(path)

        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # Set global variable old waypoints in case of failure
        global old_waypoints
        old_waypoints = waypoints
        # DONE: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        try:
            self.connection.start()
        except ConnectionAbortedError:
            print("Connection timed out. Restarting...")
            self.connection.stop()
            initiate(args)

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


def initiate(args):
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=300)
    if 'drone' in locals():
        time.sleep(1)
        drone.start()
    else:
        drone = MotionPlanning(conn)
        time.sleep(1)
        drone.start()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    initiate(args)
