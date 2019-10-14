import random
import math
import matplotlib.pyplot as plt
import numpy as np
import copy


class RRT_planner:
    """
    Rapid Random Tree (RRT) planner
    """

    class Node:
        """
        Node for RRT
        """
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.parent = None
            self.path_x = []
            self.path_y = []

    def __init__(self, start, goal, list_obstacles, rand_area,
                 max_branch_length=0.5, path_res=0.1, goal_sample_rate=5, max_iter=1000):
        """
        Parameters:
            start: Start Position [x,y]
            goal: Goal Position [x,y]
            list_obstacles: obstacle Positions [[x,y,size],...]
            rand_area: random Sampling Area [x_min, x_max, y_min, y_max]
            max_branch_length : maximal extension for one step
            path_res : resolution of obstacle checking in the path
            goal_sample_rate : percentage of samples that are artifically set to the goal
            max_iter: maximal number of iterations

        """
        self.start_node = self.Node(start[0], start[1])
        self.end_node = self.Node(goal[0], goal[1])
        self.rand_area = rand_area
        self.max_branch_length = max_branch_length
        self.path_res = path_res
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.list_obstacles = list_obstacles
        self.list_nodes = []

    def plan(self, show_anim=True):
        """
        Returns the path from goal to start.
        show_anim: flag for show_anim on or off
        """

        self.list_nodes = [self.start_node]
        for it in range(self.max_iter):
            new_node = self.Node(0, 0)
            ####### 
            # Objective: create a valid new_node according to the RRT algorithm and append it to "self.list_nodes"
            # You can call any of the functions defined lower, or add your own.

            # YOUR CODE HERE
            x_rand = self.get_random_node() # Sample (x,y)
            nearest_idx = self.get_closest_node_id(self.list_nodes, x_rand) # Find node in V that is closest to the new sampled node
            new_node = self.extend(self.list_nodes[nearest_idx], x_rand) # Connect x_nearest and x_rand (the sampled node) s.t. the distance between the two is <= max_branch_length
            is_collide = self.collision(new_node, self.list_obstacles) # Check if connection between x_nearest and x_rand is obstacle free
            if is_collide == False: # If obstacle free, append new node to V, and edge to E
                self.list_nodes.append(new_node)
            #######

            if show_anim and it % 5 == 0:
                self.draw_graph(random_node)

            if self.distance_to_goal(new_node.x, new_node.y) <= self.max_branch_length:
                print("Reached goal")
                return self.make_final_path(len(self.list_nodes) - 1)

            if show_anim and it % 5:
                self.draw_graph(random_node)

        return None  # cannot find path

    def extend(self, or_node, dest_node):
        """
        Returns a new node going from or_node in the direction of dest_node with maximal distance of max_branch_length. New node path goes from parent to new node with steps of path_res.
        """
        new_node = self.Node(or_node.x, or_node.y)
        dist, angle = self.compute_dist_ang(new_node, dest_node)
        dist_extension = self.max_branch_length

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if dist_extension > dist:
            dist_extension = dist

        n_expand = math.floor(dist_extension / self.path_res)

        for _ in range(n_expand):
            new_node.x += self.path_res * math.cos(angle)
            new_node.y += self.path_res * math.sin(angle)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        dist, _ = self.compute_dist_ang(new_node, dest_node)
        if dist <= self.path_res:
            new_node.x = dest_node.x
            new_node.y = dest_node.y
            new_node.path_x[-1] = dest_node.x
            new_node.path_y[-1] = dest_node.y

        new_node.parent = or_node

        return new_node

    def draw_graph(self, rnd=None, final_path = False):
        # Draw a graph of the path
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.list_nodes:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ob_x, ob_y, size) in self.list_obstacles:
            plt.plot(ob_x, ob_y, "ok", ms=30 * size)

        if final_path:
            plt.plot([x for (x, y) in final_path], [y for (x, y) in final_path], '-r')
        plt.plot(self.start_node.x, self.start_node.y, "xr")
        plt.plot(self.end_node.x, self.end_node.y, "xr")
        plt.axis([0, 7, 0, 5])
        plt.gca().invert_yaxis()
        plt.grid(True)
        plt.pause(0.01)

    def make_final_path(self, goal_ind):
        # Returns the path as the list of all the node positions in node_list, from end to start
        path = [[self.end_node.x, self.end_node.y]]
        node = self.list_nodes[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def distance_to_goal(self, x, y):
        dx = x - self.end_node.x
        dy = y - self.end_node.y
        return math.sqrt(dx ** 2 + dy ** 2)

    def get_random_node(self):
        # Returns a random node within random area, with the goal being sampled with a probability of goal_sample_rate %
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.rand_area[0], self.rand_area[1]),
                            random.uniform(self.rand_area[2], self.rand_area[3]))
        else:  # goal point sampling
            rnd = self.Node(self.end_node.x, self.end_node.y)
        return rnd

    @staticmethod
    def collision(node, obstacleList):
        # Returns True if collision between a node and at least one obstacle in obstacleList
        for (ob_x, ob_y, size) in obstacleList:
            list_dist_x = [ob_x - x for x in node.path_x]
            list_dist_y = [ob_y - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(list_dist_x, list_dist_y)]

            if min(d_list) <= (size/2) ** 2:
                return True 

        return False     

    @staticmethod
    def compute_dist_ang(or_node, dest_node):
        # Computes distance and angle between origin and destination nodes
        dx = dest_node.x - or_node.x
        dy = dest_node.y - or_node.y
        d = math.sqrt(dx ** 2 + dy ** 2)
        angle = math.atan2(dy, dx)
        return d, angle

    @staticmethod
    def get_closest_node_id(list_nodes, random_node):
        # Returns index of node in list_nodes that is the closest to random_node
        dist_list = [(node.x - random_node.x) ** 2 + (node.y - random_node.y)
                 ** 2 for node in list_nodes]
        min_id = dist_list.index(min(dist_list))

        return min_id
    
    
    
class RTT_Path_Follower:
    """
    Follows a path given by RRT_Planner
    """
    def __init__(self, path, local_env):
        self.path = path
        self.env = local_env
        self.last_pos = None # To store the last position of the robot
        self.last_angle = None # To store the last angle of the robot
        self.angle_reached = False # Flag to indicate if desired angle has been reached
        self.last_dist = math.inf # To store the last distance between robot and the next waypoint
    
    def next_action(self):
        # Current position and angle
        cur_pos_x = self.env.cur_pos[0]
        cur_pos_y = self.env.cur_pos[2]
        cur_angle = self.env.cur_angle
        
        v = 0.
        omega = 0.
        
        #######
        #
        # YOUR CODE HERE: change v and omega so that the Duckiebot keeps on following the path
        #
        # If first point, store the current pos and angle to the last_* variables
        if self.last_pos is None:
            self.last_pos = np.array([copy.deepcopy(cur_pos_x),copy.deepcopy(cur_pos_y)]) 
            self.last_angle = copy.deepcopy(cur_angle)

        # Because we are deleting the content of self.path whenever we have reached
        # one of the waypoints, when we reach the final goal, self.path will only
        # contain the initial position of the robot before it followed the trajectory
        # so we can just delete it
        if len(self.path) <= 1:
            del self.path[0] # delete the initial position of the robot
            v = 0.
            omega = 0.
            return v, omega 

        # Calculate angle error
        x_diff = self.path[-2][0] - self.last_pos[0]
        y_diff = self.path[-2][1] - self.last_pos[1]
        desired_angle = np.arctan2(-y_diff,x_diff)
        error_angle = cur_angle%(2*np.pi) - desired_angle%(2*np.pi)
        angle_tolerance = 0.05 # If angle error is below this number, assume to have reached desired angle
        dist_tolerance = 0.02 # If distance error is below this number, assume to have reached desired position

        # If robot is not facing the waypoint, rotate the robot until it is
        if self.angle_reached == False and (math.fabs(error_angle) > angle_tolerance):
            v = 0. # Set v = 0, so the robot does not move but only rotating
            omega = -error_angle # Adjust omega to be proportional to the angle error
        elif self.angle_reached == False and (math.fabs(error_angle) <= angle_tolerance): # If angle has been reached, turn the flag on
            self.angle_reached = True # Flag if the robot is facing the waypoint

        # If robot is facing the waypoint, move forward, do not turn anymore
        if self.angle_reached:
            pos = np.array([cur_pos_x,cur_pos_y]) # Current position of robot
            dist =  np.linalg.norm(self.path[-2] - pos) # Current distance to waypoint from robot's current position
            if dist > dist_tolerance and self.last_dist > dist:
                omega = 0. # Set omega to 0 so it doesn't turn
                v = dist # Adjust velocity to be proportional to the distance left
                self.last_dist = copy.deepcopy(dist) # Update lst distance between robot and target waypoint
            else: # If the robot has reached the waypoint
                omega = 0.
                v = 0.
                # Reset all the last_* variables and angle_reached flag
                self.last_pos = np.array([copy.deepcopy(cur_pos_x),copy.deepcopy(cur_pos_y)]) 
                self.last_angle = copy.deepcopy(cur_angle)
                self.angle_reached = False
                self.last_dist = math.inf
                del self.path[-2] # Remove the waypoint that we just reached
        #######
        
        return v, omega
    
