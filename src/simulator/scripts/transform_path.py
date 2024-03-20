#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import Float32, Float32MultiArray
import tf2_ros
import tf2_py
from tf2_geometry_msgs import do_transform_point
import numpy as np
from scipy.spatial import cKDTree
import csv
import os


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_global = self.create_publisher(Path, '/global_path', 10)
        self.publisher_local = self.create_publisher(Path, '/planning/front_path/offset_path', 10)
        self.publisher_vel = self.create_publisher(Float32, '/planning/desired_velocity', 10)
        self.publish_vel_array = self.create_publisher(Float32MultiArray, 'velocities', 10) #future velocities
        self.publish_curvature = self.create_publisher(Float32MultiArray, '/planning/front_path/curvature', 10)

        self.global_path = Path()
        #self.global_path.header.frame_id = 'map'
        self.global_path.header.frame_id = 'map'

        input_file = self.declare_parameter('csv_file', './src/simulator/maps/raceline_traj_with_velocity_monza_edited.csv').value
        # Positions will be used to build KDTree
        self.positions = []
        self.velocities = []
        self.curvatures = []
        self.vel_scalar = 1.0
        self.prev_vel = 0

        self.num_laps = 0
        self.crossing_line = False
        self.lap_start_time = self.get_clock().now()

        if input_file:
            data_x_y_v_k = self.read_specific_columns(input_file)
            self.assign_data_to_path(data_x_y_v_k)
        else:
            self.straight_line()
        
        #self.straight_line()

        self.publisher_global.publish(self.global_path) 
    
        # KDTree allows for quick lookup of points
        self.kdtree = cKDTree(self.positions)

        # Setup storage for TF buffer and define listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transform = None

        timer_period = 0.01  # seconds
        # Timer to call our publish function
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def straight_line(self):
        for i in range(10000):
            x = i * 0.1
            y = 0.0
            pose = PoseStamped()
            pose.header.frame_id = 'map' #tells us which frame this path is in
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.global_path.poses.append(pose)
            self.positions.append([x, y])
    
    def assign_data_to_path(self, data):
        #print("assigning csv data to path msg")
        #print(data)

        #WORK ON ADDING ORIENTATION?
        for x, y, v, k in data:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.global_path.poses.append(pose)
            self.positions.append([x, y])
            self.velocities.append(v)
            self.curvatures.append(k)


    def read_specific_columns(self, input_file):
        with open(input_file, 'r') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            header = next(reader)  # Read the header row
            values = [(float(row[0]), float(row[1]), float(row[2]), float(row[3])) for row in reader] # x, y, vel_des, 
            #print(values)
        return values #return all the values within these columns
    

    def timer_callback(self):
        # Try to get transform vehicle->map, return if fails
        
        try:
            self.transform = self.tf_buffer.lookup_transform("map", "vehicle", rclpy.time.Time()) 
        except Exception as e:
            print(e)
            return

        orig = PointStamped()
        orig.point.x, orig.point.y = 0.0, 0.0

        # Transforming (0, 0) in car frame to global frame gives global car coordinates
        # there has to be a better way to do this
        car_loc = do_transform_point(orig, self.transform)
        x, y = car_loc.point.x, car_loc.point.y

        # Find closest point on path to global point
        _, idx = self.kdtree.query([x, y])
    
        global_path = Path()
        global_path.header.frame_id = 'map'

        local_path = Path()
        local_path.header.frame_id = 'vehicle'

        if (idx + 400) % len(self.positions) < idx and (idx + 350) % len(self.positions) > idx:
            self.crossing_line = True
        else:
            if self.crossing_line:
                self.crossing_line = False
                self.num_laps += 1
                if self.num_laps > 0:
                    print(f"Lap: {self.num_laps}, Time: {(self.get_clock().now() - self.lap_start_time).nanoseconds/1e9}, velocity scalar: {self.vel_scalar}")
                #self.vel_scalar += 0.1
                self.lap_start_time = self.get_clock().now()

        velocities = Float32MultiArray()
        curvatures = Float32MultiArray()

        for i in range(idx, idx+100):
            i = i % len(self.global_path.poses)
            pose = self.tf_buffer.transform(self.global_path.poses[i], 'vehicle')
            local_path.poses.append(pose)
            velocities.data.append(self.velocities[i]*self.vel_scalar)
            curvatures.data.append(self.curvatures[i])

            pose2 = self.global_path.poses[i]
            global_path.poses.append(pose2)


        
        vel_msg = Float32()
        vel_msg.data = self.velocities[idx]



        # Publish local path and global path (just to refresh, even though it doesn't change)
        self.publisher_vel.publish(vel_msg)
        self.publisher_local.publish(local_path)          

        #start = self.get_clock().now()
        #self.publisher_global.publish(self.global_path)
        self.publish_vel_array.publish(velocities)
        self.publish_curvature.publish(curvatures)

        #print(self.get_clock().now()-start)



def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)

    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

