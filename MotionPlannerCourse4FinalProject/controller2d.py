#!/usr/bin/env python3
"""
Assignments Solution Author: Engin Bozkurt
Motion Planning for Self-Driving Cars
Aug 24, 2019
"""

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('v_error_integral', 0.0)
        self.vars.create_var('heading_error_integral', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            # in this controller, we assume no braking, so brake_output is always 0
            # We use PID + feedforward method for longitudinal controller
            # the dynamic model is not used here, just pure tuning of the gains
            kp = 0.2
            ki = 0.05
            kd = 0.01

            v_error = v_desired - v

            dt = t - self.vars.t_previous
            v_error_derivative = v_error/dt

            self.vars.v_error_integral += v_error * dt

            feedback = kp * v_error + ki * self.vars.v_error_integral + kd * v_error_derivative

            # calculate the feedforward(predicted) throttle, the data is collect by
            # running CARLA simulation at diffferent throttle level and measuring car speed
            look_ahead = waypoints[len(waypoints)-1]
            v_desired_forward = look_ahead[2]
            if v_desired_forward <= 6:
            	feedforward = 0.15 + v_desired_forward/6*(0.6-0.15)
            elif v_desired <= 11.5:
            	feedforward = 0.6 + (v_desired_forward-6)/(11.5-6)*(0.8-0.6)
            else:
            	feedforward = 0.8 + (v_desired_forward-11.5)/85
            
            throttle_output = feedforward + feedback
            throttle_output = min(throttle_output,1)
            throttle_output = max(throttle_output,0)

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            brake_output    = 0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            # in this controller, we use pure pursuit method to design lateral controller
            # the dynamic model is not used here, just pure tuning of the gains.

            L = 1.5
            kp_lat = 1.5
            ki_lat = 0.2
            kd_lat = 0.5
            # use the middle point in the given waypoints as the look ahead target
            look_ahead_index = len(waypoints)//2
            look_ahead = waypoints[look_ahead_index]

            # heading_error = yaw - np.arctan2(waypoints[look_ahead_index][1] - waypoints[look_ahead_index-1][1], waypoints[look_ahead_index][0] - waypoints[look_ahead_index-1][0])
            heading_error = yaw - np.arctan2(waypoints[1][1] - waypoints[0][1], waypoints[1][0] - waypoints[0][0])
            while heading_error > np.pi: heading_error -= np.pi*2
            while heading_error < -np.pi: heading_error += np.pi*2
            #print("heading_error: ", heading_error/3.1415926*180)

            heading_error_derivative = heading_error/dt
            self.vars.heading_error_integral += heading_error * dt
            feedback_lateral = kp_lat * heading_error + ki_lat * self.vars.heading_error_integral + kd_lat * heading_error_derivative


            ld = np.sqrt((look_ahead[0] - x)**2 + (look_ahead[1] - y)**2)

            vector_look_ahead = [look_ahead[0] - x,look_ahead[1] - y]
            vector_car = [np.cos(yaw),np.sin(yaw)]
            corss_track_error = np.cross(vector_look_ahead, vector_car)

            curvature = 2/ld/ld*corss_track_error
            
            # Change the steer output with the lateral controller. 
            steer_output = np.arctan(curvature*L) + feedback_lateral
            steer_output = -steer_output
            steer_output = min(steer_output,1.22)
            steer_output = max(steer_output,-1.22)
            #print("steer_output: ",steer_output/3.1415926*180)

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)
            #print(throttle_output,steer_output)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t  # Store forward speed to be used in next step
