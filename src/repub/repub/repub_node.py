import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Int8, Header
from blackandgold_msgs.msg import BasestationCommand
from novatel_oem7_msgs.msg import BESTPOS, HEADING2, BESTVEL, INSPVA
from autonoma_msgs.msg import VehicleInputs, PowertrainData
from deep_orange_msgs.msg import PtReport

from pyproj import CRS, Transformer 

from geometry_msgs.msg import TransformStamped
from math import cos, sin
from tf2_ros import TransformBroadcaster
import math
import numpy as np

#Node to republish messages from Simulink stack to SIM Race Simulator

class Simulink_to_SIM_Race(Node):
    curr_acc_cmd = 0.0
    curr_brake_cmd = 0.0
    curr_steer_cmd = 0.0
    curr_gear_cmd= 0

    throttle_counter = 0
    brake_counter = 0
    steering_counter = 0
    gear_counter = 0

    engine_rpm = 0.0
    current_gear = 0

    STEER_CONSTANT = 13.3333 #200/15


    true_easting = 521921.53 #Float32 UTM 32N, origin of Monza 
    true_northing = 5051752.75 #UTM 32N
    true_heading = 0.0 #Float32 degrees
    true_local_x = 0.0
    true_local_y = 0.0

    true_vel = 0.0

    
    def __init__(self):
        super().__init__('repub_node')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Float32,
            # '/joystick/accelerator_cmd',
            '/joystick/throttle_cmd',
            self.accelerator_cmd_callback,
            10
        )
        self.subscription = self.create_subscription(
            Float32,
            '/joystick/brake_cmd',
            self.curr_brake_cmd_callback,
            10
        )
        self.subscription = self.create_subscription(
            Float32,
            '/joystick/steering_cmd',
            self.steering_cmd_callback,
            10
        )
        self.subscription = self.create_subscription(
            Int8,
            '/joystick/gear_cmd',
            self.gear_cmd_callback,
            10
        )

        self.subscription = self.create_subscription(
            PowertrainData,
            '/powertrain_data',
            self.powertrain_callback,
            10
        )

        # self.subscription = self.create_subscription(
        #     BESTPOS, 
        #     '/novatel_top/bestgnsspos', 
        #     self.pos_callback, 
        #     10)

        # self.subscription = self.create_subscription(
        #     HEADING2, 
        #     '/novatel_bottom/heading2',  #novatel_top/heading2 appears to be broken
        #     self.heading_callback, 
        #     10)   

        self.subscription = self.create_subscription(
            BESTVEL, 
            '/novatel_top/bestvel', 
            self.vel_callback, 
            10)

        self.subscription = self.create_subscription(
            INSPVA, 
            '/novatel_top/inspva', 
            self.pos_heading_callback, 
            10)


        #PUBLISHERS
        self.cmd_input_publisher = self.create_publisher(
            VehicleInputs,
            '/vehicle_inputs',
            10
        )
        self.vel_publisher = self.create_publisher(
            Float32,
            '/localization/vehicle_speed', 
            10)
        self.engine_rpm_publisher = self.create_publisher(
            PtReport,
            '/raptor_dbw_interface/pt_report',
            10)


        # create timer to publish every second
        self.timer1 = self.create_timer(0.01, self.publish_vehicle_inputs)
        self.timer2 = self.create_timer(0.01, self.publish_tf)
        self.timer3 = self.create_timer(0.01, self.publish_vel)
        self.timer4 = self.create_timer(0.01, self.publish_engine_rpm)

        
    def powertrain_callback(self, msg):
        self.engine_rpm = msg.engine_rpm
        self.current_gear = msg.current_gear

    def accelerator_cmd_callback(self, msg):
        self.curr_acc_cmd = msg.data
        #print(self.curr_acc_cmd)
        #curr_gear_cmd= int(round(msg.data))  # Convert all msg data to int
        #self.publish_vehicle_inputs(throttle_cmd=msg.data, curr_brake_cmd=msg.data, steering_cmd=msg.data, gear_cmd=gear_cmd)
        #self.publish_vehicle_inputs()
        #self.get_logger().info('Republished accelerator cmd based on joystick')

    def curr_brake_cmd_callback(self,msg):
        self.curr_brake_cmd = msg.data

    def steering_cmd_callback(self,msg):
        steer_cmd = msg.data
        self.curr_steer_cmd = steer_cmd * 13.5

    def gear_cmd_callback(self,msg):
        self.curr_gear_cmd= msg.data

    # def pos_callback(self,msg):
    #     latitude = msg.lat
    #     longitude = msg.lon
    #     [self.true_easting, self.true_northing] = self.latlon_to_utm(latitude, longitude)
    #     self.true_local_x = self.true_easting - 521921.53
    #     self.true_local_y = self.true_northing - 5051752.75
    #     # self.get_logger().info('Lat:' + str(latitude))
    #     # self.get_logger().info('Long:' + str(longitude))
    #     # self.get_logger().info('Easting:' + str(self.true_easting))
    #     # self.get_logger().info('Northing:' + str(self.true_northing))

    def vel_callback(self,msg):
        self.true_vel = msg.hor_speed #m/s
        #self.get_logger().info('Got velocity (m/s):' + str(self.true_vel))

    # def heading_callback(self,msg):
    #     self.true_heading = self.degrees_to_radians(msg.heading) #* 0.0174533 + (-1 * 3.1415)#AKA Yaw (deg -> radians)
    #     self.get_logger().info('Got heading (radians):' + str(self.true_heading))

    def pos_heading_callback(self, msg):
        self.true_heading = -1* self.degrees_to_radians(msg.azimuth) - math.pi/2
        lat = msg.latitude
        long = msg.longitude
        [self.true_easting, self.true_northing] = self.latlon_to_utm(lat, long)
        self.true_local_x = self.true_easting - 521921.53
        self.true_local_y = self.true_northing - 5051752.75
        


    #callback for timer 1
    def publish_vehicle_inputs(self):
        msg = VehicleInputs()
        msg.header = Header()

        msg.throttle_cmd = self.curr_acc_cmd # Percent
        # msg.throttle_cmd = 100. # Percent       
        # self.throttle_counter = self.throttle_counter + 1
        # msg.throttle_cmd_count = self.throttle_counter

        msg.brake_cmd = self.curr_brake_cmd #kPa
        # msg.brake_cmd = 0.0
        # self.brake_counter = self.brake_counter + 1
        # msg.brake_cmd_count = self.brake_counter

        msg.steering_cmd = self.curr_steer_cmd #degrees
        # msg.steering_cmd = 5.0 #degrees
        # self.steering_counter = self.steering_counter + 1
        # msg.steering_cmd_count = self.steering_counter 

        msg.gear_cmd= self.curr_gear_cmd
        #msg.gear_cmd= 2

        self.cmd_input_publisher.publish(msg)

        # self.get_logger().info('throttle %:' + str(msg.throttle_cmd))
        # self.get_logger().info('brake kPa:' + str(msg.brake_cmd))
        # self.get_logger().info('steering degrees:' + str(msg.steering_cmd))
        # self.get_logger().info('gear:' + str(msg.gear_cmd))
        # self.get_logger().info('Republished vehicle cmds')

    #callback for timer 2
    def publish_tf(self):
        
        # Replace these with your actual x, y positions, and yaw angle
        x = self.true_local_x
        y = self.true_local_y
        yaw = self.true_heading  # Yaw angle in radians
        
        t = TransformStamped()
        #assign values to corr. tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'vehicle'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0  # Assuming a 2D scenario
        q = self.quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        #self.get_logger().info('Rotation +Z:' + str(q[2]))

        self.tf_broadcaster.sendTransform(t)
        #self.get_logger().info('Published tf')

    #callback for timer 3
    def publish_vel(self):
        vel_msg = Float32()
        vel_msg.data = self.true_vel
        self.vel_publisher.publish(vel_msg)

        #self.get_logger().info('Republished velocity: ' + str(self.true_vel))
    
    def publish_engine_rpm(self):
        engine_rpm_msg = PtReport()
        engine_rpm_msg.engine_rpm = self.engine_rpm
        engine_rpm_msg.current_gear = self.current_gear
        engine_rpm_msg.engine_on_status = True
        self.engine_rpm_publisher.publish(engine_rpm_msg)

# ============== Helper Functions ================

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

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

    def degrees_to_radians(self, degrees): #[0,359] to [-pi,pi]
        radians = (degrees - 180) / 180 * math.pi
        return radians

def main(args=None):
    rclpy.init(args=args)
    node = Simulink_to_SIM_Race()
    #print("test")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
