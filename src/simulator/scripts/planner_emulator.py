#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point
import tf2_ros
import tf2_py
from tf2_geometry_msgs import do_transform_point
import numpy as np

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_global = self.create_publisher(Path, '/global_path', 10)
        self.publisher_local = self.create_publisher(Path, '/local_path', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transform = None

    def timer_callback(self):

        self.transform = self.tf_buffer.lookup_transform("vehicle", "map", rclpy.time.Time())
        global_path_msg = Path()
        global_path_msg.header.frame_id = 'map'

        local_path_msg = Path()
        local_path_msg.header.frame_id = 'vehicle'
        
        for i in range(50):
            global_x = i * 0.1
            global_y = i * 0.05

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = global_x
            pose.pose.position.y = global_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            global_path_msg.poses.append(pose)

            point_stamped = PointStamped()
            point_stamped.header.frame_id = "map"
            point_stamped.point.x = global_x
            point_stamped.point.y = global_y

            transformed_point = do_transform_point(point_stamped, self.transform).point

            pose = PoseStamped()
            pose.header.frame_id = 'vehicle'
            pose.pose.position.x = transformed_point.x
            pose.pose.position.y = transformed_point.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            local_path_msg.poses.append(pose)

            self.publisher_global.publish(global_path_msg)
            self.publisher_local.publish(local_path_msg)

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)

    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

