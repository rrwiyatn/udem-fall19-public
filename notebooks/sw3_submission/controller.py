import numpy as np

class Controller():
    def __init__(self):
        self.gain = 2.0
        self.last_dist = None
        self.last_angle = None
        self.integral_dist = 0.0
        self.integral_angle = 0.0
        pass

    def angle_control_commands(self, dist, angle):
        # Return the angular velocity in order to control the Duckiebot so that it follows the lane.
        # Parameters:
        #     dist: distance from the center of the lane. Left is negative, right is positive.
        #     angle: angle from the lane direction, in rad. Left is negative, right is positive.
        # Outputs:
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        
        omega = 0. 
        
        #######
        #
        # MODIFY ANGULAR VELOCITY
        #
        # YOUR CODE HERE
        #
        v = 0.5
        k_theta = 20.0
        theta_threshold = np.pi / 6.
        kd = -(k_theta**2) / (4.*v)
        d_threshold = abs((k_theta * theta_threshold) / kd)
        if dist < -d_threshold:
            sat = -d_threshold
        elif dist > d_threshold:
            sat = d_threshold
        else:
            sat = dist
        error_dist = -sat
        error_angle = angle
        omega = (kd * error_dist) + (k_theta * error_angle)

        #######
        
        return  omega

    
    def pid_control_commands(self, dist, angle, kp_dist, kp_angle, kd_dist, kd_angle, ki_dist, ki_angle):
        # Return the angular velocity in order to control the Duckiebot so that it follows the lane.
        # Parameters:
        #     dist: distance from the center of the lane. Left is negative, right is positive.
        #     angle: angle from the lane direction, in rad. Left is negative, right is positive.
        #     kp_dist: constant for proportial term with distance error
        #     kp_angle: constant for proportial term with angle error
        #     kd_dist: constant for derivative term with distance error
        #     kd_angle: constant for derivative term with angle error
        #     ki_dist: constant for integral term with distance error
        #     ki_angle: constant for integral term with angle error
        # Outputs:
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        if self.last_dist is None: # If first reading, use only P-term
            omega = (kp_dist * dist) + (kp_angle * angle)
        else:
            diff_dist = dist - self.last_dist
            diff_angle = angle - self.last_angle
            omega = (kp_dist * dist) + (kp_angle * angle) + (kd_dist * diff_dist) + (kd_angle * diff_angle) + (ki_dist * self.integral_dist) + (ki_angle * self.integral_angle)
        self.integral_dist += dist
        self.integral_angle += angle
        self.last_dist = dist
        self.last_angle = angle
        return omega


    def wrap_angle(self, angle):
        # Return the angle such that it is in the range [0,+2pi]
        # Parameters:
        #     angle: angle in rad
        # Outputs:
        #     new_angle: angle wrapped in the range [0,+2pi]
        new_angle = angle % (2 * np.pi)
        return new_angle
    

    def pure_pursuit(self, env, pos, angle, follow_dist=0.25):
        # Return the angular velocity in order to control the Duckiebot using a pure pursuit algorithm.
        # Parameters:
        #     env: Duckietown simulator
        #     pos: global position of the Duckiebot
        #     angle: global angle of the Duckiebot
        # Outputs:
        #     v: linear veloicy in m/s.
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        
        
        closest_curve_point = env.unwrapped.closest_curve_point
        
        # Find the curve point closest to the agent, and the tangent at that point
        closest_point, closest_tangent = closest_curve_point(pos, angle)

        iterations = 0
        
        lookup_distance = follow_dist
        multiplier = 0.5
        curve_point = None
        
        while iterations < 10:            
            ########
            #
            # TODO 1: Modify follow_point so that it is a function of closest_point, closest_tangent, and lookup_distance
            #
            ########
            follow_point = closest_point + (lookup_distance * closest_tangent)
            
            curve_point, _ = closest_curve_point(follow_point, angle)

            # If we have a valid point on the curve, stop
            if curve_point is not None:
                break

            iterations += 1
            lookup_distance *= multiplier
        ########
        #
        # TODO 2: Modify omega
        #
        ######## 
        duck_to_point = curve_point - pos
        dist = np.linalg.norm(duck_to_point)
        unit_duck_to_point = duck_to_point / dist
        angle_between_x_axis_and_target = np.arctan2(-duck_to_point[2], duck_to_point[0])
        alpha = angle - angle_between_x_axis_and_target
        K = 0.1 # Gain assuming L = K * V (making the lookup distance as a function of some gain times the velocity)
        omega = -(np.sin(alpha)) / (K) # Since we write L = K * V, the equation for omega is simplified to this.
        v = 0.5
        
        crosstrack_error = np.sin(alpha) * dist # For plotting crosstrack error
        angle_error = self.wrap_angle(np.arctan2(-closest_tangent[2],closest_tangent[0])) - self.wrap_angle(angle) # For plotting angle error in notebook
        if angle_error > np.pi:
            angle_error = 2 * np.pi - angle_error
        return v, omega, crosstrack_error, angle_error