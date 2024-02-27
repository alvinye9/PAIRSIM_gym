#!/usr/bin/env python3

import rclpy
import yaml
import os
import math
from pyproj import CRS, Transformer 
from argparse import Namespace
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32, Float32MultiArray, Int8
from novatel_oem7_msgs.msg import INSPVA

from cvxopt import matrix, solvers
from .waypoint_follow import PurePursuitPlanner 
from .waypoint_follow import get_actuation


from . import waypoint_follow 

import numpy as np
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt

class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Get the directory of the current Python file
        current_dir = os.path.dirname(__file__)
        
        # Construct the full path to the configuration file
        config_file_path = os.path.join(current_dir, 'config_monza_map.yaml')

        with open(config_file_path) as file:
            conf_dict = yaml.load(file, Loader=yaml.FullLoader)
            conf = Namespace(**conf_dict)

        self.declare_parameter('wb', value = 0.17145 + 0.15875) #wb length, prob meters
        wheel_base = self.get_parameter('wb').value

        # create instance of pure pursuit planner
        self.PPP = PurePursuitPlanner(conf, wheel_base)
        # self.get_logger().info(str(self.PPP.max_reacquire)) #debugging, works
        # waypoints_shape = np.shape(self.PPP.waypoints) #debugging, works 
        # self.get_logger().info(str(waypoints_shape)) #debugging, works 

        # self.path_subscription = self.create_subscription(
        #     Path,
        #     '/planning/front_path/offset_path',
        #     self.path_cb, #current x, y?
        #     10
        # )

        # self.vel_array_subscription = self.create_subscription(
        #     Float32MultiArray,
        #     '/velocities', #reference velocities
        #     self.vel_array_cb,
        #     10
        # )

        # self.vel_array_subscription = self.create_subscription(
        #     Float32,
        #     '/localization/vehicle_speed',
        #     self.speed_cb,
        #     10
        # )

        # self.wheel_odom_subscription = self.create_subscription(
        #     Odometry,
        #     '/odometry/wheel_odom',
        #     self.wheel_odom_cb,
        #     10
        # )

        # self.spin_mon_subscription = self.create_subscription(
        #     Float32,
        #     '/spin_monitor_test',
        #     self.spin_mon_cb,
        #     10
        # )

        self.subscription = self.create_subscription(
            INSPVA, 
            '/novatel_top/inspva', 
            self.pos_heading_cb, 
            10)
        

        self.declare_parameter('prediction_ts', value=0.2)
        self.declare_parameter('lateral_error_weight', value=20.0)
        self.declare_parameter('lateral_error_velocity_weight', value=3.0)
        self.declare_parameter('R_weight', value=0.5)
        self.declare_parameter('horizon_length', value=12)

        self.declare_parameter('tlad', value = 0.82461887897713965) #lookahead distance
        self.declare_parameter('mass', value = 3.463388126201571) 
        self.declare_parameter('vgain', value = 1.0) #velocity gain
        self.declare_parameter('lf', value = 0.15597534362552312) #idk what this is lmao
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        

        self.publisher_steer_cmd = self.create_publisher(Float32, '/joystick/steering_cmd', 10)
        self.publisher_throttle_cmd = self.create_publisher(Float32, '/joystick/throttle_cmd', 10)
        self.publisher_brake_cmd = self.create_publisher(Float32, '/joystick/brake_cmd', 10)
        self.publisher_gear_cmd = self.create_publisher(Int8, '/joystick/gear_cmd', 10)

        self.path_pub = self.create_publisher(Path, '/mpc_debug_path', 10)
        self.desired_w_pub = self.create_publisher(Float32, '/desired_yaw_rate', 10)
        
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.update)

        param_timer_period = 1.0
        self.param_timer = self.create_timer(param_timer_period, self.update_params)

        self.horizon = self.get_parameter('horizon_length').value
        self.ts = self.get_parameter('prediction_ts').value

        self.curr_path = []
        self.F = np.array([[1, self.ts],
                    [0, 1]])
        self.G = np.array([self.ts**2/2, self.ts])


        self.lateral_error_weight = self.get_parameter('lateral_error_weight').value
        self.lateral_error_velocity_weight = self.get_parameter('lateral_error_velocity_weight').value
        self.R_weight = self.get_parameter('R_weight').value
        

        self.current_v = 0
        self.current_yaw_rate = 0
        self.spin_mon = 0
        self.current_vs = []
        self.poses = []
        self.steer_angle = 0

        # self.construct_matrix()
        self.ys = np.zeros(self.horizon*2)

        self.current_path = []
        self.kdtree = None
        self.states = []
        self.inputs = []
        self.vels = []
        self.yaw_errors = []

    # def get_current_heading(self, path_msg):
    #     # if not path_msg.poses:
    #     #     return None
    #     if path_msg is None or not path_msg.poses:
    #         raise ValueError("Invalid Path message. Cannot extract current heading, very sad beans.")
    
    #     # Get the first pose in the path
    #     pose = path_msg.poses[0].pose
        
    #     # Extract the orientation quaternion
    #     orientation = pose.orientation
        
    #     # Convert quaternion to Euler angles
    #     x = orientation.x
    #     y = orientation.y
    #     z = orientation.z
    #     w = orientation.w
        
    #     # Calculate roll, pitch, and yaw (heading) from quaternion
    #     roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    #     pitch = math.asin(2 * (w * y - z * x))
    #     yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        
    #     # Convert yaw to degrees
    #     heading_degrees = np.degrees(yaw)
        
    #     return heading_degrees


    #useful skeleton but currently not used
    def update_params(self):
        lateral_error_weight = self.get_parameter('lateral_error_weight').value
        lateral_error_velocity_weight = self.get_parameter('lateral_error_velocity_weight').value
        R_weight = self.get_parameter('R_weight').value
        horizon = self.get_parameter('horizon_length').value
        ts = self.get_parameter('prediction_ts').value


    # unused, but potentially useful callback functions
        
    # def vel_array_cb(self, msg):
    #     self.current_vs = msg.data

    # def speed_cb(self, msg: Float32):
    #     self.current_v = msg.data
    
    # def wheel_odom_cb(self, msg):
    #     self.current_yaw_rate = msg.twist.twist.angular.z

    # def spin_mon_cb(self, msg):
    #     self.spin_mon = msg.data

    # def path_cb(self, msg: Path): #update x, y (in vehicle frame), yaw (check if yaw is correct)
    #     self.current_path.clear()
    #     for pose in msg.poses:
    #         position = pose.pose.position
    #         self.current_path.append([position.x, position.y])
    #         # self.get_logger().info(str(position.x))
    #         # self.get_logger().info(str(position.y))


    #     # self.lookahead_point = self.PPP._get_current_waypoint(
    #     #     waypoints = self.PPP.waypoints, #FIXME this is using PPP's waypoint array. Maybe better to have it use the one we generated?
    #     #     lookahead_distance = self.get_parameter('tlad').value, 
    #     #     position = self.position,
    #     #     theta = None #seems to not be needed?
    #     # )
        
    
    #     # self.kdtree = cKDTree(self.current_path)

# ============== CRS Helper Functions ================

    def latlon_to_utm(self, latitude, longitude):
        # Define UTM projection for Zone 32N
        utm_zone = 32
        utm_band = 'N'  # Northern Hemisphere
        utm_proj = CRS.from_string(f'EPSG:{32600 + utm_zone}')

        # Create a transformer
        transformer = Transformer.from_crs(CRS.from_epsg(4326), utm_proj, always_xy=True)

        # Transform latitude and longitude to UTM coordinates
        utm_easting, utm_northing = transformer.transform(longitude, latitude)

        return utm_easting, utm_northing

    def degrees_to_radians(self, degrees): #[0,359] to [-pi,pi]
        radians = (degrees - 180) / 180 * math.pi
        return radians
# ========================================================
       
       # callback function to update poses (x y WRT vehicle frame and yaw WRT map frame in ENU)
    def pos_heading_cb(self, msg):
        self.true_heading = -1* self.degrees_to_radians(msg.azimuth) - math.pi/2
        lat = msg.latitude
        long = msg.longitude
        [self.true_easting, self.true_northing] = self.latlon_to_utm(lat, long)
        self.true_local_x = self.true_easting - 521921.53 #origin of map
        self.true_local_y = self.true_northing - 5051752.75

        self.pose_x = self.true_local_x
        self.pose_y = self.true_local_y 
        self.pose_theta = self.true_heading

    # def find_closest_point(self, robot_state):
    #     # Query the closest point to the robot's position
    #     # print(robot_state)
    #     _, index = self.kdtree.query([robot_state[0], robot_state[1]])
        
    #     closest_point = self.current_path[index]
        
    #     return closest_point, index

    # def normalize_angle(self, yaw):
    #     while yaw > np.pi / 2:
    #         yaw -= np.pi
    #     while yaw < -np.pi / 2:
    #         yaw += np.pi
    #     return yaw
        
    # def calculate_path_heading(self, closest_idx):
    #     # Calculate the heading based on the closest point and its neighbors
    #     if closest_idx == 0:
    #         next_point = self.current_path[closest_idx + 1]
    #         closest_point = self.current_path[closest_idx]
    #     elif closest_idx == len(self.current_path) - 1:
    #         next_point = self.current_path[closest_idx]
    #         closest_point = self.current_path[closest_idx - 1]
    #     else:
    #         next_point = self.current_path[closest_idx + 1]
    #         closest_point = self.current_path[closest_idx - 1]
            
    #     delta_x = next_point[0] - closest_point[0]
    #     delta_y = next_point[1] - closest_point[1]
        
    #     theta_path = np.arctan2(delta_y, delta_x)
        
    #     return self.normalize_angle(theta_path)


    
    # def get_error(self, x, y, yaw):
    #     robot_state = [x, y, yaw]
    #     closest_point, closest_idx = self.find_closest_point(robot_state)
    #     theta_path = self.calculate_path_heading(closest_idx)
    #     e_theta = self.normalize_angle(robot_state[2] - theta_path)
    #     e_y = (robot_state[1] - closest_point[1]) * np.cos(e_theta) - (robot_state[0] - closest_point[0]) * np.sin(e_theta)

    #     if e_y > 2.0:
    #         #print(e_y, e_theta)
    #         pass

    #     return e_y, e_theta, self.current_vs[closest_idx]
    

    def update(self):
        pose_x = self.pose_x #must be updated from AWSIM
        pose_y = self.pose_y
        pose_theta = self.pose_theta

        tlad = self.get_parameter('tlad').value #can be updated from waypoint_follow.py
        vgain = self.get_parameter('vgain').value

        speed, steering_angle = self.PPP.plan(pose_x, pose_y, pose_theta, tlad, vgain)
        # self.get_logger().info('speed_cmd: ' + str(speed))
        # self.get_logger().info('steering_cmd: ' + str(steering_angle))
        # self.steer_angle = np.rad2deg(2.9718 * w / (max(self.current_vs[0], 1)))

        steer_gain = 1.0
        steer_msg = Float32()
        steer_msg.data = steering_angle * steer_gain

        throttle_gain = 0.5
        throttle_msg = Float32()
        throttle_msg.data = speed * throttle_gain

        self.publisher_steer_cmd.publish(steer_msg)
        self.publisher_throttle_cmd.publish(throttle_msg)




def main(args=None):
    rclpy.init(args=args)
    # create instance of ros2 node, all relevant f1tenth functions can be executed repeatedly via ROS2 callbacks
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()


