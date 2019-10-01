import numpy as np
import time

class Controller():
    def __init__(self):
        self.gain = 2.0
        # self.last_error_dist = None
        self.integral_term = 0.0
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
        k_theta = 10.0
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


    def pid_angle_control_commands(self, dist, angle):
        # Return the angular velocity in order to control the Duckiebot so that it follows the lane.
        # Parameters:
        #     dist: distance from the center of the lane. Left is negative, right is positive.
        #     angle: angle from the lane direction, in rad. Left is negative, right is positive.
        # Outputs:
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        current_time = time.time()
        v = 0.5
        k_theta = 10.0
        ki = 0.5
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
        # self.last_error_dist = error_dist
        self.integral_term += sat
        omega = (kd * error_dist) + (k_theta * error_angle) + (ki * self.integral_term)
        # omega = (kd * error_dist) + (ki * self.integral_term)
        return omega
    

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
            #TODO 1: Modify follow_point so that it is a function of closest_point, closest_tangent, and lookup_distance
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
        #TODO 2: Modify omega
        #
        ######## 
        duck_to_point = curve_point - pos
        dist = np.linalg.norm(duck_to_point)
        unit_duck_to_point = duck_to_point / dist
        dir_unit_vector = np.array([np.sin(angle), 0, np.cos(angle)])
        alpha = np.arccos(np.dot(dir_unit_vector, unit_duck_to_point))
        K = 0.2
        omega = -(np.cos(alpha)) / (K) # Scaling dist with speed
        v = 0.5
        
        

        return v, omega