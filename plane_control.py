# -*- coding: utf-8 -*-
import numpy as np
from enum import Enum


class LongitudinalAutoPilot(object):
    def __init__(self):
        self.max_throttle_rpm = 2500

        self.min_throttle = 0.0
        self.max_throttle = 1.0

        self.min_pitch_cmd = -10.0 * np.pi / 180.0
        self.max_pitch_cmd = 30.0 * np.pi / 180.0
        self.max_pitch_cmd_climb = 45 * np.pi / 180.0

        self.speed_int = 0.0
        self.alt_int = 0.0
        self.climb_speed_int = 0.0

        # Gain parameters for pitch_loop PD controller
        self.kp_pitch = 4.0
        self.kp_q = 0.5

        # Gain parameters for altitude_loop PI controller
        self.kp_alt = 0.03
        self.ki_alt = 0.02

        # Gain parameters for airspeed_loop PI controller
        self.kp_speed = 0.3
        self.ki_speed = 0.3

        # Feed-forward term for throttle
        self.throttle_feedforward = 0.658

        # Gain parameters for airspeed_pitch_loop PI controller
        self.kp_speed_pitch = -0.2
        self.ki_speed_pitch = -0.2

        # Threshold for switching behavior on altitude matching
        self.altitude_switch = 20

        return

    """Used to calculate the elevator command required to achieve the target
    pitch
    
        Args:
            pitch: in radians
            pitch_rate: in radians/sec
            pitch_cmd: in radians
        
        Returns:
            elevator_cmd: in percentage elevator [-1,1]
    """
    def pitch_loop(self, pitch, pitch_rate, pitch_cmd):
        # Pitch loop should be implemented as a PD controller
        # Desired pitch rate is zero

        pitch_error = pitch_cmd - pitch
        pitch_rate_error = 0 - pitch_rate

        elevator_cmd = self.kp_pitch * pitch_error + self.kp_q * pitch_rate_error

        # Limit elevator_cmd to [-1, 1] interval
        if elevator_cmd > 1:
            elevator_cmd = 1
        elif elevator_cmd < -1:
            elevator_cmd = -1

        return elevator_cmd

    """Used to calculate the pitch command required to maintain the commanded
    altitude
    
        Args:
            altitude: in meters (positive up)
            altitude_cmd: in meters (positive up)
            dt: timestep in seconds
        
        Returns:
            pitch_cmd: in radians
    """
    def altitude_loop(self, altitude, altitude_cmd, dt):
        # Altitude loop should be implemented as a PI controller

        altitude_error = altitude_cmd - altitude
        self.alt_int += altitude_error * dt

        pitch_cmd_unbound = self.kp_alt * altitude_error + self.ki_alt * self.alt_int

        if pitch_cmd_unbound > self.max_pitch_cmd:
            pitch_cmd = self.max_pitch_cmd
        elif pitch_cmd_unbound < self.min_pitch_cmd:
            pitch_cmd = self.min_pitch_cmd
        else:
            pitch_cmd = pitch_cmd_unbound

        # Integrator anti-windup
        if self.ki_alt != 0 and pitch_cmd != pitch_cmd_unbound:
            self.alt_int += dt / self.ki_alt * (pitch_cmd - pitch_cmd_unbound)

        return pitch_cmd

    """Used to calculate the throttle command required command the target 
    airspeed
        
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
            dt: timestep in seconds
        
        Returns:
            throttle_command: in percent throttle [0,1]
    """
    def airspeed_loop(self, airspeed, airspeed_cmd, dt):
        # Airspeed loop should be implemented as a PI controller with feed-forward

        airspeed_error = airspeed_cmd - airspeed
        self.speed_int += airspeed_error * dt

        throttle_cmd_unbound = self.kp_speed * airspeed_error \
            + self.ki_speed * self.speed_int \
            + self.throttle_feedforward

        if throttle_cmd_unbound > self.max_throttle:
            throttle_cmd = self.max_throttle
        elif throttle_cmd_unbound < self.min_throttle:
            throttle_cmd = self.min_throttle
        else:
            throttle_cmd = throttle_cmd_unbound

        # Integrator anti-windup
        if self.ki_speed != 0 and throttle_cmd != throttle_cmd_unbound:
            self.speed_int += dt / self.ki_speed * (throttle_cmd - throttle_cmd_unbound)

        return throttle_cmd

    """Used to calculate the pitch command required to maintain the commanded
    airspeed
    
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
            dt: timestep in seconds
        
        Returns:
            pitch_cmd: in radians
    """
    def airspeed_pitch_loop(self, airspeed, airspeed_cmd, dt):
        # Airspeed pitch loop should be implemented as a PI controller

        airspeed_error = airspeed_cmd - airspeed
        self.climb_speed_int += airspeed_error * dt

        pitch_cmd_unbound = self.kp_speed_pitch * airspeed_error + self.ki_speed_pitch * self.climb_speed_int

        if pitch_cmd_unbound > self.max_pitch_cmd_climb:
            pitch_cmd = self.max_pitch_cmd_climb
        else:
            pitch_cmd = pitch_cmd_unbound

        # Integrator anti-windup
        if self.ki_speed_pitch != 0 and pitch_cmd != pitch_cmd_unbound:
            self.climb_speed_int += dt / self.ki_speed_pitch * (pitch_cmd - pitch_cmd_unbound)

        return pitch_cmd

    """Used to calculate the pitch command and throttle command based on the
    aicraft altitude error
    
        Args:
            airspeed: in meter/sec
            altitude: in meters (positive up)
            airspeed_cmd: in meters/sec
            altitude_cmd: in meters/sec (positive up)
            dt: timestep in seconds
            
        Returns:
            pitch_cmd: in radians
            throttle_cmd: in in percent throttle [0,1]
    """
    def longitudinal_loop(self, airspeed, altitude, airspeed_cmd, altitude_cmd,
                          dt):

        if altitude < altitude_cmd - self.altitude_switch:
            # Climb at max throttle
            pitch_cmd = self.airspeed_pitch_loop(airspeed, airspeed_cmd, dt)
            throttle_cmd = self.max_throttle
            # Adjust pitch at min throttle
        elif altitude > altitude_cmd + self.altitude_switch:
            pitch_cmd = self.airspeed_pitch_loop(airspeed, airspeed_cmd, dt)
            throttle_cmd = self.min_throttle
        else:
            # Use altitude controller
            throttle_cmd = self.airspeed_loop(airspeed, airspeed_cmd, dt)
            pitch_cmd = self.altitude_loop(altitude, altitude_cmd, dt)

        return[pitch_cmd, throttle_cmd]


class LateralAutoPilot:

    def __init__(self):
        self.g = 9.81
        self.integrator_yaw = 0.0

        self.gate = 1
        self.max_roll = 60 * np.pi / 180.0
        self.state = 1

        # Gain parameters for roll_attitude_hold_loop PD controller
        self.kp_roll = 10.0
        self.kd_roll = 1.0

        # Gain parameters for the sideslip_hold_loop PI controller
        self.kp_sideslip = -2.0
        self.ki_sideslip = -1.0

        # Helper parameters for sideslip_hold_loop PI integrator
        self.integrator_beta = 0.0
        self.beta_error_last = 0.0

        # Gain parameters for the yaw_hold_loop PI controller
        self.kp_yaw = 2.1
        self.ki_yaw = 0.4

        # Gain parameters for the straight_line_guidance P controller
        self.kp_course = -0.003

        # Gain parameters for the orbit_guidance P controller
        self.kp_orbit_guidance = 2.5

        return

    """Used to limit angles to values between -pi and pi
    
       Args:
           theta: angle to limit between -pi and pi
           
       Returns:
           value between -pi and pi
    """
    @staticmethod
    def fmod(theta):
        return theta if abs(theta) < np.pi else (theta + np.pi) % (2 * np.pi) - np.pi

    """Used to calculate the commanded aileron based on the roll error
    
        Args:
            phi_cmd: commanded roll in radians
            phi: roll angle in radians
            roll_rate: in radians/sec
            T_s: timestep in sec
            
        Returns:
            aileron: in percent full aileron [-1,1]
    """
    def roll_attitude_hold_loop(self,
                                phi_cmd,  # commanded roll
                                phi,    # actual roll
                                roll_rate,
                                T_s = 0.0):
        # Implemented as a PD controller.  Explicitly not adding an integral term, no use for Ts

        phi_error = phi_cmd - phi
        rate_error = -roll_rate
        aileron = self.kp_roll * phi_error + self.kd_roll * rate_error

        return aileron

    """Used to calculate the commanded roll angle from the course/yaw angle
    
        Args:
            yaw_cmd: commanded yaw in radians
            yaw: roll angle in radians
            T_s: timestep in sec
            roll_ff: in radians/sec
            
        Returns:
            roll_cmd: commanded roll in radians
    """
    def yaw_hold_loop(self,
                         yaw_cmd,  # desired heading
                         yaw,     # actual heading
                         T_s,
                         roll_ff=0):
        # Implemented as a PI controller with feed-forward value.

        yaw_error = LateralAutoPilot.fmod(yaw_cmd - yaw)
        self.integrator_yaw += yaw_error * T_s

        roll_cmd_unbound = self.kp_yaw * yaw_error \
            + self.ki_yaw * self.integrator_yaw \
            + roll_ff
        roll_cmd_unbound = LateralAutoPilot.fmod(roll_cmd_unbound)

        if roll_cmd_unbound > self.max_roll:
            roll_cmd = self.max_roll
        elif roll_cmd_unbound < -self.max_roll:
            roll_cmd = -self.max_roll
        else:
            roll_cmd = roll_cmd_unbound

        # Integrator anti-windup
        if self.ki_yaw != 0 and roll_cmd != roll_cmd_unbound:
            self.integrator_yaw += T_s / self.ki_yaw * LateralAutoPilot.fmod(roll_cmd - roll_cmd_unbound)

        return roll_cmd

    """Used to calculate the commanded rudder based on the sideslip
    
        Args:
            beta: sideslip angle in radians
            T_s: timestep in sec
            
        Returns:
            rudder: in percent full rudder [-1,1]
    """
    def sideslip_hold_loop(self,
                           beta, # sideslip angle
                           T_s):
        # Implemented as a PI controller; sideslip should be commanded to zero

        beta_error = -beta

        # Implementing trapeizodal integration
        self.integrator_beta += (beta_error + self.beta_error_last) * T_s / 2
        self.beta_error_last = beta_error

        rudder_unbound = self.kp_sideslip * beta_error + self.ki_sideslip * beta_error

        if rudder_unbound > 1:
            rudder = 1
        elif rudder_unbound < -1:
            rudder = -1
        else:
            rudder = rudder_unbound

        # Integrator anti-windup
        if self.ki_sideslip != 0 and rudder != rudder_unbound:
            self.integrator_beta += T_s / self.ki_sideslip * (rudder - rudder_unbound)

        return rudder

    """Used to calculate the desired course angle based on cross-track error
    from a desired line
    
        Args:
            line_origin: point on the desired line in meters [N, E, D]
            line_course: heading of the line in radians
            local_position: vehicle position in meters [N, E, D]
            
        Returns:
            course_cmd: course/yaw cmd for the vehicle in radians
    """
    def straight_line_guidance(self, line_origin, line_course,
                               local_position):

        # Compute the distance between the local position and its projection on the line
        v_n = line_origin[0] - local_position[0]
        v_e = line_origin[1] - local_position[1]

        proj_angle = np.arctan2(v_e, v_n) - line_course
        line_distance = np.sqrt(v_n**2 + v_e**2) * np.sin(proj_angle)

        # Compute distance as angle from arbitrary point on line
        angle_error = np.arctan2(self.kp_course * line_distance, 1)

        # Control the error to zero
        course_cmd = line_course - angle_error

        return course_cmd

    """Used to calculate the desired course angle based on radius error from
    a specified orbit center
    
        Args:
            orbit_center: in meters [N, E, D]
            orbit_radius: desired radius in meters
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            clockwise: specifies whether to fly clockwise (increasing yaw)
            
        Returns:
            course_cmd: course/yaw cmd for the vehicle in radians
    """
    def orbit_guidance(self, orbit_center, orbit_radius, local_position, yaw,
                       clockwise = True):

        # Determine the vector from orbit center to plane location
        center_to_location_n = local_position[0] - orbit_center[0]
        center_to_location_e = local_position[1] - orbit_center[1]

        # Get vector size and magnitude
        center_to_location_distance = np.sqrt(center_to_location_n**2 + center_to_location_e**2)
        center_to_location_orientation = np.arctan2(center_to_location_e, center_to_location_n)

        # Compute the error between actual orbit and desired orbit
        distance_error = center_to_location_distance - orbit_radius
        # Express error as angle / function of orbit
        angle_error = np.arctan(self.kp_orbit_guidance * distance_error / orbit_radius)
        if not clockwise:
            angle_error = -angle_error

        # Compute tangent route orientation as feedforward -- rotate to get tangent from vector from orbit center
        if clockwise:
            tangent_angle = center_to_location_orientation + np.pi / 2
        else:
            tangent_angle = center_to_location_orientation - np.pi / 2

        # Combine feedforward and error and limit between -pi and pi
        course_cmd = angle_error + tangent_angle
        course_cmd = LateralAutoPilot.fmod(course_cmd)

        return course_cmd

    """Used to calculate the feedforward roll angle for a constant radius
    coordinated turn
    
        Args:
            speed: the aircraft speed during the turn in meters/sec
            radius: turning radius in meters
            cw: true=clockwise turn, false = counter-clockwise turn
            
        Returns:
            roll_ff: feed-forward roll in radians
    """
    def coordinated_turn_ff(self, speed, radius, cw):
        # Compute roll from Coriolis force

        roll_ff = np.sqrt(speed ** 2 / (self.g * radius))

        # Flip sign if not clockwise
        if not cw:
            roll_ff = -roll_ff

        return roll_ff

    """Used to calculate the desired course angle and feed-forward roll
    depending on which phase of lateral flight (orbit or line following) the 
    aicraft is in
    
        Args:
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            airspeed_cmd: in meters/sec
            
        Returns:
            roll_ff: feed-forward roll in radians
            yaw_cmd: commanded yaw/course in radians
    """
    def path_manager(self, local_position, yaw, airspeed_cmd):

        # Initialize roll and yaw commands
        roll_ff = 0
        yaw_cmd = 0

        # Pick the next target gate, fall through if gate reached
        if self.gate == 1:
            target = [500, 20]
            distance_sq = (local_position[0] - target[0]) ** 2 + (local_position[1] - target[1]) ** 2
            if distance_sq < 100:
                self.gate = 2
                self.integrator_yaw = 0
            else:
                line_origin = [0, 20]
                line_course = 0
                yaw_cmd = self.straight_line_guidance(line_origin=line_origin,
                                                      line_course=line_course,
                                                      local_position=local_position)
                roll_ff = 0

        if self.gate == 2:
            target = [900, -380]
            distance_sq = (local_position[0] - target[0]) ** 2 + (local_position[1] - target[1]) ** 2
            if distance_sq < 100:
                self.gate = 3
                self.integrator_yaw = 0
            else:
                orbit_center = [500, -380]
                orbit_radius = 400
                clockwise = False
                yaw_cmd = self.orbit_guidance(orbit_center=orbit_center,
                                              orbit_radius=orbit_radius,
                                              yaw=yaw,
                                              clockwise=clockwise,
                                              local_position=local_position)
                roll_ff = self.coordinated_turn_ff(speed=airspeed_cmd, radius=orbit_radius, cw=clockwise)

        if self.gate == 3:
            target = [600, -680]
            distance_sq = (local_position[0] - target[0]) ** 2 + (local_position[1] - target[1]) ** 2
            if distance_sq < 100:
                self.gate = 4
                self.integrator_yaw = 0
            else:
                orbit_center = [600, -380]
                orbit_radius = 300
                clockwise = False
                yaw_cmd = self.orbit_guidance(orbit_center=orbit_center,
                                              orbit_radius=orbit_radius,
                                              yaw=yaw,
                                              clockwise=clockwise,
                                              local_position=local_position)
                roll_ff = self.coordinated_turn_ff(speed=airspeed_cmd, radius=orbit_radius, cw=clockwise)

        if self.gate == 4:
            target = [100, -680]
            distance_sq = (local_position[0] - target[0]) ** 2 + (local_position[1] - target[1]) ** 2
            if distance_sq < 100:
                self.gate = 5
                self.integrator_yaw = 0
            else:
                line_origin = [600, -680]
                line_course = np.pi
                yaw_cmd = self.straight_line_guidance(line_origin=line_origin,
                                                      line_course=line_course,
                                                      local_position=local_position)
                roll_ff = 0

        if self.gate > 4:
            # fly to the endpoint
            line_origin = [-400, -680]
            line_course = np.pi
            yaw_cmd = self.straight_line_guidance(line_origin=line_origin,
                                                  line_course=line_course,
                                                  local_position=local_position)
            roll_ff = 0

        return roll_ff, yaw_cmd

    """Used to calculate the desired course angle and feed-forward roll
    depending on which phase of lateral flight (orbit or line following) the 
    aicraft is in
    
        Args:
            waypoint_tuple: 3 waypoints, (prev_waypoint, curr_waypoint, next_waypoint), waypoints are in meters [N, E, D]
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            airspeed_cmd: in meters/sec
            
        Returns:
            roll_ff: feed-forward roll in radians
            yaw_cmd: commanded yaw/course in radians
            cycle: True=cycle waypoints (at the end of orbit segment)
    """
    def waypoint_follower(self, waypoint_tuple, local_position, yaw, airspeed_cmd):
        roll_ff = 0.0
        yaw_cmd = 0.0
        cycle = False

        turning_radius = 500

        # Extract waypoints
        w1 = waypoint_tuple[0][0:2]
        w2 = waypoint_tuple[1][0:2]
        w3 = waypoint_tuple[2][0:2]

        # Unit vectors for the current leg (w1 -> w2) and the next leg (w2 -> w3)
        u12 = (w2 - w1) / np.linalg.norm(w2 - w1)
        u23 = (w3 - w2) / np.linalg.norm(w3 - w2)

        # Extract the angle between unit vectors
        c = -np.dot(u12, u23)
        angle = np.arccos(np.clip(c, -1, 1))

        # Project the radius into the waypoint legs to obtain the distance between w2 and the orbit center
        tangent_to_waypoint = turning_radius / np.tan(angle / 2)

        if self.state == 1:
            # Line following

            # Compute first tangent point
            t1 = w2 - tangent_to_waypoint * u12

            if np.dot(local_position - t1, u12) > 0:
                # Hyperplane through tangent has been crossed, switch states to orbit following
                self.state = 2
                self.integrator_yaw = 0

            else:
                # Follow line from w1 to w2
                line_origin = w1
                line_course = np.arctan2(w2[1] - w1[1], w2[0] - w1[0])

                # Controls to follow line
                yaw_cmd = self.straight_line_guidance(line_origin=line_origin,
                                                      line_course=line_course,
                                                      local_position=local_position)
                roll_ff = 0

        elif self.state == 2:
            # Orbit following

            # Compute second tangent poinnt
            t2 = w2 + tangent_to_waypoint * u23

            if np.dot(local_position - t2, u23) > 0:
                # Hyperplane through tangent has been crossed, switch states to line following
                self.state = 1
                self.integrator_yaw = 0

                # Turn on flag to switch to next waypoint set
                cycle = True

            else:
                # Orbit around center
                w2_center_size = tangent_to_waypoint / np.cos(angle/2)
                w2_center_unit = (u23 - u12) / np.linalg.norm(u23 - u12)
                orbit_center = w2 + w2_center_size * w2_center_unit

                # Clockwise iff angle between u12, u13 is less than np.pi
                clockwise = (u12[0] * u23[1] - u12[1] * u23[0] > 0)

                # Controls to follow orbit
                yaw_cmd = self.orbit_guidance(orbit_center=orbit_center,
                                              orbit_radius=turning_radius,
                                              local_position=local_position,
                                              yaw=yaw,
                                              clockwise=clockwise)
                roll_ff = self.coordinated_turn_ff(speed=airspeed_cmd, radius=turning_radius, cw=clockwise)

        return roll_ff, yaw_cmd, cycle


class FlyingCarPlanner:

    def __init__(self):
        self.takeoff_altitude_before_turn = 20
        self.landing_altitude_after_turn = 20

        self.flight_altitude = 100
        self.decelerate_distance = 100

        self.max_yaw_error = 1 * np.pi / 180
        self.max_vtol_xyz_error = 5
        self.max_vtol_speed_error = 0.1

        self.max_flight_xyz_error = 5

        return

    class WaypointCommand(Enum):
        TAKEOFF = 1
        VTOL = 2
        FLIGHT = 3
        LANDING = 4

    """Used to create a series of waypoints for a flying car path between both locations.
    
        Args:
            start_location: start position for the vehicle in meters [N, E, D]
            start_yaw: start yaw for the vehicle, in radians
            end_location: end position for the vehicle in meters [N, E, D]
            end_yaw: end yaw for the vehicle, in radians
            
        Returns:
            waypoints: array of dictionary objects, each with one of the following forms:
                * {cmd: WaypointCommand.TAKEOFF, altitude: d}
                * {cmd: WaypointCommand.VTOL, position: [n, e, d], yaw = yaw}
                * {cmd: WaypointCommant.FLIGHT, position: [n, e, d]]
                * {cmd: Waypoing.LANDING}
    """
    def build_waypoints(self, start_location, start_yaw, end_location, end_yaw):
        # A flight path consists of the following waypoints:
        #  - TAKEOFF to safe altitude
        #  - VTOL to flight altitude rotating to correct flight heading
        #  - Fly towards destination
        #  - VTOL decelerate to a position above the end location, at flight altitude
        #  - VTOL down to safe landing altitude, rotating to correct yaw
        #  - VTOL down without rotation to land

        waypoints = []

        trajectory = end_location - start_location
        trajectory_unit = trajectory / np.linalg.norm(trajectory)

        flight_line_yaw = np.arctan2(trajectory[1], trajectory[0])
        deceleration_start = end_location - self.decelerate_distance * trajectory_unit

        # Lift to initial altitude
        safe_takeoff_altitude = start_location[2] + self.takeoff_altitude_before_turn
        waypoints.append({
            'cmd': FlyingCarPlanner.WaypointCommand.TAKEOFF,
            'altitude': safe_takeoff_altitude
        })

        # Turn to target yaw
        waypoints.append({
            'cmd': FlyingCarPlanner.WaypointCommand.VTOL,
            'position': [start_location[0], start_location[1], self.flight_altitude],
            'yaw': flight_line_yaw
        })

        # Fly to next waypoint
        waypoints.append({
            'cmd': FlyingCarPlanner.WaypointCommand.FLIGHT,
            'position': [deceleration_start[0], deceleration_start[1], self.flight_altitude]
        })

        # Decelerate
        waypoints.append({
            'cmd': FlyingCarPlanner.WaypointCommand.VTOL,
            'position': [end_location[0], end_location[1], self.flight_altitude],
            'yaw': flight_line_yaw
        })

        # VTOL lower and rotate
        safe_landing_altitude = end_location[2] + self.landing_altitude_after_turn
        if safe_landing_altitude > self.flight_altitude:
            safe_landing_altitude = self.flight_altitude

        waypoints.append({
            'cmd': FlyingCarPlanner.WaypointCommand.VTOL,
            'position': [end_location[0], end_location[1], safe_landing_altitude],
            'yaw': end_yaw
        })

        # VTOL land
        waypoints.append({
            'cmd': FlyingCarPlanner.WaypointCommand.LANDING
        })

        return waypoints

    """Used to determine whether the provided waypoint has been reached
    
        Args:
            target_waypoint: a waypoint dict as returned by build_waypoints
            local_position: the current position of the vehicle in meters [N, E, D]
            yaw: the current yaw of the vehicle
            airspeed: the current airspeed of the vehicle
            
        Returns:
            boolean, whether the waypoint has been reached
    """
    def _waypoint_reached(self, target_waypoint, local_position, yaw, airspeed):

        waypoint_cmd = target_waypoint['cmd']

        if waypoint_cmd == FlyingCarPlanner.WaypointCommand.TAKEOFF:
            # Check distance
            if abs(local_position[2] - target_waypoint['altitude']) > self.max_vtol_xyz_error:
                return False

        elif waypoint_cmd == FlyingCarPlanner.WaypointCommand.VTOL:
            # Check speed
            if abs(airspeed) > self.max_vtol_speed_error:
                return False

            # Check yaw
            waypoint_yaw = target_waypoint['yaw']
            if abs(FlyingCarPlanner.fmod(waypoint_yaw - yaw)) > self.max_yaw_error:
                return False

            # Check distance
            waypoint_ned = target_waypoint['position']
            if np.linalg.norm(waypoint_ned - local_position) > self.max_vtol_xyz_error:
                return False

            return True

        # Check distance
        elif waypoint_cmd == FlyingCarPlanner.WaypointCommand.FLIGHT:
            waypoint_ned = target_waypoint['position']
            if np.linalg.norm(waypoint_ned - local_position) > self.max_vtol_xyz_error:
                return False

        return True

    """Used for selecting the controls between for a given waypoint 
       
        Args:
            target_waypoint: current waypoint target, as returned by build_waypoints
            local_position: current vehicle position, in meters [N, E, D]
            yaw: current vehicle yaw, in radians
            airspeed: current vehicle speed, in m/s
            
        Returns:
            flying_car_cmd: one of:
                - takeoff instructions [altitude] 
                - VTOL flight command [north, east, altitude, holding]
                - plane flight command [line_origin, line_course]
                - landing instructions (None)
            cycle: True=cycle waypoints (at the end of orbit segment)
    """
    def waypoint_follower(self, target_waypoint, local_position, yaw, airspeed):
        flying_car_cmd = None
        cycle = False

        waypoint_cmd = target_waypoint['cmd']

        if self._waypoint_reached(target_waypoint, local_position, yaw, airspeed):
            cycle = True

        elif waypoint_cmd == FlyingCarPlanner.WaypointCommand.TAKEOFF:
            flying_car_cmd = target_waypoint['altitude']

        elif waypoint_cmd == FlyingCarPlanner.WaypointCommand.VTOL:
            target_position = target_waypoint['position']
            target_heading = target_waypoint['yaw']
            flying_car_cmd = [target_position[0], target_position[1], target_position[2], target_heading]

        elif waypoint_cmd == FlyingCarPlanner.WaypointCommand.FLIGHT:
            line_origin = local_position
            line_vector = target_waypoint['position'] - local_position
            line_course = np.arctan(line_vector[1], line_vector[0])
            flying_car_cmd = [line_origin, line_course]

        return flying_car_cmd, cycle

    """Used to limit angles to values between -pi and pi
    
       Args:
           theta: angle to limit between -pi and pi
           
       Returns:
           value between -pi and pi
    """
    @staticmethod
    def fmod(theta):
        return theta if abs(theta) < np.pi else (theta + np.pi) % (2 * np.pi) - np.pi


def euler2RM(roll,pitch,yaw):
    R = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
    cr = np.cos(roll)
    sr = np.sin(roll)

    cp = np.cos(pitch)
    sp = np.sin(pitch)

    cy = np.cos(yaw)
    sy = np.sin(yaw)

    R[0,0] = cp*cy
    R[1,0] = -cr*sy+sr*sp*cy
    R[2,0] = sr*sy+cr*sp*cy

    R[0,1] = cp*sy
    R[1,1] = cr*cy+sr*sp*sy
    R[2,1] = -sr*cy+cr*sp*sy

    R[0,2] = -sp
    R[1,2] = sr*cp
    R[2,2] = cr*cp

    return R.transpose()
