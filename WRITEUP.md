# Optional Project: Fixed-Wing Control

---

#### Fixed Wing Equations
* The original cheatsheet with equations for this project is [available online](https://www.overleaf.com/read/cvqmtzyhqjnj).
* A [rendered PDF](Fixed%20Wing%20Cheatsheet.pdf) has been included in this repository.

---

## Longitudinal Scenarios

### Scenario #1: Trim (Unity Only)

![Trim Scenario Intro](images/scenario/scenario1_intro.png)

This scenario requires no coding or gains to tune -- just calibrating the values manually on the flight simulator.

The fixed throttle at **65.8%** eventually leads to success:

![Trim Scenario Success](images/scenario/scenario1_success.png)


### Scenario #2: Altitude Hold

![Altitude Hold Scenario Intro](images/scenario/scenario2_intro.png)


This scenario requires implementing the pitch hold and altitude hold controllers.  They are implemented as PD and PI controllers, respectively.

![Altitude Diagram](Diagrams/altitude_hold.png)

![Pitch Altitude Hold Control Loop](images/control_loop/pitch_altitude.png)

![Altitude Hold Control Loop](images/control_loop/altitude.png)

Tuning the process produces the following gains:

```
    # Gain parameters for pitch_loop PD controller
    self.kp_pitch = 4.0
    self.kp_q = 0.5

    # Gain parameters for altitude_loop PI controller
    self.kp_alt = 0.03
    self.ki_alt = 0.02
```

Of note, the official solution (and the tuning facility through Unity!) has a couple of implementation choices that cause a discrepancy between the tuning obtained in Unity and the initial implementation I had:
- A minimum pitch command is set to -10 degrees, different from the maximum pitch command of 30 degrees.
- A particular implementation of integrator anti-windup is used on the PI controller for attitude_loop:

```
    # Integrator anti-windup
    if(gain_i_alt != 0):
        self.alt_int = self.alt_int + dt/gain_i_alt*(pitch_cmd-pitch_cmd_unsat)
``` 
 
This implementation changes the integrated error to have been updated by a value equivalent to the error that would have been added if the commanded pitch had been its saturated value, rather than the unbounded one.

I implemented equivalent logic, for ease of tuning in future scenarios.

![Altitude Hold Scenario Success](images/scenario/scenario2_success.png)

### Scenario #3: Airspeed Hold

![Airspeed Hold Scenario Intro](images/scenario/scenario3_intro.png)

This scenario requires implementing a PI controller for the airspeed hold based on throttle. 

![Airspeed_Hold Diagram](Diagrams/airspeed_hold.png)

![Airspeed Hold Control Loop](images/control_loop/airspeed_throttle.png)

The code implementation is *almost* analogous to the implementation for the PI controller on the altitude loop -- we include a feed-forward term based on the throttle obtained for scenario 1.

Initial tuning provides the following gain values:

```
    # Gain parameters for airspeed_loop PI controller
    self.kp_speed = 0.4
    self.ki_speed = 0.3

    # Feed-forward term for throttle
    self.throttle_feedforward = 0.658
```

This works! ... after making sure that the code was running on a sufficiently fast computer, with the relevant graphic options set (i.e. disabling G-SYNC on my NVIDEA GTX card.)

![Airspeed Hold Scenario Success](images/scenario/scenario3_success.png)

### Scenario #4: Steady Climb

![Climb Scenario Intro](images/scenario/scenario4_intro.png)

This requires yet another PI controller -- this time, for the climb state; controlling the airspeed hold based on pitch.

![Airspeed Pitch Hold Diagram](Diagrams/airspeed_pitch_hold.png)

![Airspeed Pitch Hold Control Loop](images/control_loop/airspeed_pitch.png)

The Unity / manual tuning of parameters seems to not work at all for this scenario -- tuning had to be done by running the python script and changing its parameters there instead.

```
    # Gain parameters for airspeed_pitch_loop PI controller
    self.kp_speed_pitch = -0.2
    self.ki_speed_pitch = -0.2
```

These parameters are *negative*, unlike all of the gains seen so far -- the commanded pitch here is expected to be negative when climbing.

![Climb Scenario Success](images/scenario/scenario4_success.png)


### Scenario #5: Longitudinal Challenge

![Longitudinal Challenge Scenario Intro](images/scenario/scenario5_intro.png)

This is relatively straightforward -- there are three potential states:

* If the altitude is too low, climb -- set throttle to max, and use airspeed_pitch_loop to control the pitch
* If the altitude is too high, 'climb down' -- set throttle to min, and use airspeed_pitch_loop to control the pitch
* Otherwise, control the throttle with airspeed_loop and the pitch with pitch_loop

Testing produced catastrophic crashes at threshold of 50m, the plane was not able to stay within the 'good enough' band with a threshold of 10m, and a threshold of 20m proved sufficient:

```
    # Threshold for switching behavior on altitude matching
    self.altitude_switch = 20
```

![Longitudinal Challenge Scenario Success](images/scenario/scenario5_success.png)

---

## Lateral / Directional Scenarios


### Scenario #6: Stabilized Roll Angle

![Stabilized Roll Angle Scenario Intro](images/scenario/scenario6_intro.png)

One more PD controller; the intended roll speed is always 0.

![Stabilized Roll Angle Diagram](Diagrams/roll_loop.png)

![Roll Altitude Roll Control Loop](images/control_loop/roll_altitude.png)

Tuning allows for a controller with gains as aggressive as Kp = 10, Ki = 1; these parameters work with the partially implemented script as well.
```
    # Gain parameters for roll_attitude_hold_loop PD controller
    self.kp_roll = 10.0
    self.kd_roll = 1.0
```

![Stabilized Roll Angle Scenario Success](images/scenario/scenario6_success.png)


### Scenario #7: Coordinated Turn

![Coordinated Turn Scenario Intro](images/scenario/scenario7_intro.png)

This scenario calls for a PI controller on beta -- the sideslip value, which should be controlled back to zero.

![Coordinated Turn Diagram](Diagrams/sideslip_hold.PNG)

![Sideslip Control Loop](images/control_loop/sideslip.png)

Initial tuning provided the following gains:

```
    # Gain parameters for the sideslip_hold_loop PI controller
    self.kp_sideslip = -2.0
    self.ki_sideslip = -1.0
```

As suggested on the scenario, however, there is occasional instability, potentially caused by difference in integration steps.  This is remediated by implenting [trapezoidal integration](https://en.wikipedia.org/wiki/Trapezoidal_rule):

```
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
```

![Coordinated Turn Scenario Success](images/scenario/scenario7_success.png)

### Scenario #8: Constant Course/Yaw Hold

![Constant Course/Yaw Hold Scenario Intro](images/scenario/scenario8_intro.png)

This scenario calls for a PI controller for the yaw.

![Constant Course/Yaw Hold Diagram](Diagrams/course_hold.PNG)

![Course Control Loop](images/control_loop/course.png)

Note: The documentation for the `yaw_hold_loop` method initially referred to parameter roll_rate, which does not exist, and does not include documentation for the feed forward parameter, which does exist.  The documentation has been updated on the solution.

Extra care being taken to keep the yaw within the -pi to pi range -- all angles (but not angular velocities) are converted to values between -pi and pi on this implementation, with the help of the function defined below as a static method in `LateralAutoPilot`. 

```
    """Used to limit angles to values between -pi and pi
    
       Args:
           theta: angle to limit between -pi and pi
           
       Returns:
           value between -pi and pi
    """
    @staticmethod
    def fmod(theta):
        return theta if abs(theta) < np.pi else (theta + np.pi) % (2 * np.pi) - np.pi
```

Tuning using Unity also failed to work for this scenario -- the behavior seems unresponsive to the UI parameters.  Tuning was instead performed using my own implementation.

```
    # Gain parameters for the yaw_hold_loop PI controller
    self.kp_yaw = 2.1
    self.ki_yaw = 0.4
```

![Constant Course/Yaw Hold Scenario Success](images/scenario/scenario8_success.png)

### Scenario #9: Straight Line Following

### Scenario #10: Orbit Following

### Scenario #11: Lateral/Directional Challenge

## Final Challenges

### Scenario #12: Full 3D Challenge

### Scenario #13: Flying Car Challenge