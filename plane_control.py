# -*- coding: utf-8 -*-
import numpy as np

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
        course_cmd = 0
        # STUDENT CODE HERE


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
        course_cmd = 0
        # STUDENT CODE HERE


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

        roll_ff = 0
        # STUDENT CODE HERE


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

        roll_ff = 0
        yaw_cmd = 0
        # STUDENT CODE HERE


        return(roll_ff,yaw_cmd)


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

        # STUDENT CODE HERE



        return(roll_ff, yaw_cmd, cycle)



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
