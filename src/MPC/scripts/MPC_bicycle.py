#!/usr/bin/env python3

import rclpy
import csv
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import Float32, Float32MultiArray
from cvxopt import matrix, solvers

import numpy as np
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
class MPC(Node):

    def __init__(self, ts, horizon):
        super().__init__('MPC_node')
        self.path_subscription = self.create_subscription(
            Path,
            '/planning/front_path/offset_path',
            self.path_cb,
            10
        )

        self.vel_array_subscription = self.create_subscription(
            Float32MultiArray,
            '/velocities',
            self.vel_array_cb,
            10
        )

        self.vel_array_subscription = self.create_subscription(
            Float32,
            '/localization/vehicle_speed',
            self.speed_cb,
            10
        )

        self.wheel_odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/wheel_odom',
            self.wheel_odom_cb,
            10
        )

        self.spin_mon_subscription = self.create_subscription(
            Float32,
            '/spin_monitor_test',
            self.spin_mon_cb,
            10
        )


        self.declare_parameter('Iz', value=1000.0) # moment of inertia kg*m^2
        self.declare_parameter('l_F', value=1.7) #m
        self.declare_parameter('l_R', value=1.2)
        self.declare_parameter('m', value=787.0) #kg
        self.declare_parameter('prediction_ts', value=0.2)
        self.declare_parameter('lateral_error_weight', value=20.0)
        self.declare_parameter('lateral_error_velocity_weight', value=3.0)
        self.declare_parameter('R_weight', value=0.5)
        self.declare_parameter('horizon_length', value=15)
        self.declare_parameter('D_f', value=0.9)  # pacejka coeff
        self.declare_parameter('D_r', value=1.0)
        self.declare_parameter('B_f', value=13.0)
        self.declare_parameter('B_r', value=11.0)
        self.declare_parameter('C_f', value=1.4)
        self.declare_parameter('C_r', value=1.4)

        self.publisher_steer_cmd = self.create_publisher(Float32, '/joystick/steering_cmd', 10)
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
        self.l_F = self.get_parameter('l_F').value
        self.l_R = self.get_parameter('l_R').value
        self.m = self.get_parameter('m').value
        self.D_f = self.get_parameter('D_f').value
        self.D_r = self.get_parameter('D_r').value
        self.B_f = self.get_parameter('B_f').value
        self.B_r = self.get_parameter('B_r').value
        self.C_f = self.get_parameter('C_f').value
        self.C_r = self.get_parameter('C_r').value
        self.Iz = self.get_parameter('Iz').value
        

        self.current_v = 0
        self.current_v_lat = 0
        self.current_yaw_rate = 0
        self.spin_mon = 0
        self.current_vs = []
        self.poses = []
        self.current_steer_angle = 0

        self.construct_matrix()
        self.ys = np.zeros(self.horizon*2)

        self.current_path = []
        self.kdtree = None
        self.states = []
        self.inputs = []
        self.vels = []
        
        self.yaw_errors = []

    def construct_matrix(self):
        self.L, self.M = self.construct_matrices(self.F, self.G, self.horizon)
        self.Q = np.eye(2*self.horizon)
        self.R = np.eye(self.horizon) 
        self.R2 = np.eye(self.horizon)
        self.eta = np.zeros(self.horizon)

        M = self.M
        Q = self.Q
        Q[::2] *= self.lateral_error_weight 
        Q[1::2] *= self.lateral_error_velocity_weight

        R = self.R * self.R_weight
        self.matrix = -np.linalg.inv((M.T @ Q @ M + R)) #@ M.T @ Q
    
    def update_params(self):
        lateral_error_weight = self.get_parameter('lateral_error_weight').value
        lateral_error_velocity_weight = self.get_parameter('lateral_error_velocity_weight').value
        R_weight = self.get_parameter('R_weight').value
        horizon = self.get_parameter('horizon_length').value
        ts = self.get_parameter('prediction_ts').value

        if (lateral_error_weight != self.lateral_error_weight 
            or lateral_error_velocity_weight != self.lateral_error_velocity_weight
            or R_weight != self.R_weight 
            or self.horizon != horizon 
            or self.ts != ts):

            self.horizon = horizon
            self.ts = ts
            self.lateral_error_weight = lateral_error_weight
            self.lateral_error_velocity_weight = lateral_error_velocity_weight
            self.R_weight = R_weight
            self.ys = np.zeros(self.horizon*2)
            self.construct_matrix()

    def construct_matrices(self, F, G, p):
        # Dimensions
        n = F.shape[0] # assuming F is square
        m = G.shape[0]

        # Initialize L and M matrices
        L = np.zeros((n * (p), n))
        M = np.zeros((p*m, p))

        # Populate L matrix
        for i in range(p-1):
            L[i*n:(i+1)*n, :] = np.linalg.matrix_power(F, i+1)

        # Populate G Matrix
        for i in range(p):
            for j in range(i+1):
                if i == j:
                    M[i*2:i*2+2, j] = G
                    continue
                M[i*2:i*2+2, j] = np.linalg.matrix_power(F, i-j) @ G


        return L, M
    
    def cal_frt_slip_agl(self, v_long, v_lat, w,steer_angle):
        
        alpha_f = np.arctan((v_lat + self.l_F * w) / v_long) - steer_angle
        alpha_r = np.arctan((v_lat - self.l_R * w) / v_long)
        return alpha_f, alpha_r
    
    def cal_lat_force(self, v_long, v_lat, w,steer_angle):
        alpha_f, alpha_r = self.cal_frt_slip_agl(v_long, v_lat, w, steer_angle)
        F_Nf = self.m * 9.8 * self.l_F / (self.l_F + self.l_R) # Normal force on front wheels
        F_Nr = self.m * 9.8 * self.l_R / (self.l_F + self.l_R) # Normal force on rear wheels
        
        F_yf = F_Nf * self.D_f * np.sin(self.C_f * np.arctan(self.B_f * alpha_f))
        F_yr = F_Nr * self.D_r * np.sin(self.C_r * np.arctan(self.B_r * alpha_r))
        return F_yf, F_yr
    
    def cal_d_yawrate (self, F_yf,F_yr w,F_xf, steer_angle):
        d_r = (1/self.Iz)* (self.l_F*(F_yf*np.cos(steer_angle)+ F_xf*np.sin(steer_angle)) - self.l_R*F_yr)
        b = steer_angle
        
        return d_r
        
    def bicycle_model(self, v_long,v_lat, w, state, steer_angle):
        dt = self.ts
        x, y, yaw = state
        print("state: ", state)
        F_yf, F_yr = self.cal_lat_force(v_long, v_lat,w,steer_angle)
       
        dv_lat = (1/self.m)*(F_yr + F_yf*np.cos(steer_angle)-self.m * v_long * w)
        
        dx = v_long * np.cos(yaw) - v_lat * np.sin(yaw)
        print("dx: ", dx)
        dy = v_long * np.sin(yaw) + v_lat * np.cos(yaw)
        print("dy: ", dy)
        dyaw = w
        print("dyaw: ", dyaw)
        new_state = state + np.array([dx, dy, dyaw])*dt
        print("new_state: ", new_state)
        v_lat = v_lat + dv_lat * dt
        return new_state, v_lat
    
    
    def rk4_step(self, v, w, state):
        dt = self.ts
        def f(state, v, w):
            x, y, yaw = state
            dyaw = w
            dx = v * np.cos(yaw)
            dy = v * np.sin(yaw)
            return np.array([dx, dy, dyaw])
        
        # Calculate the four slopes
        k1 = dt * f(state, v, w)
        k2 = dt * f(state + 0.5 * k1, v, w)
        k3 = dt * f(state + 0.5 * k2, v, w)
        k4 = dt * f(state + k3, v, w)
        
        # Combine the slopes to get the final state
        new_state = state + (k1 + 2 * k2 + 2 * k3 + k4) / 6
        
        # Normalize the yaw
        new_state[2] = self.normalize_angle(new_state[2])
        
        return new_state

    
    def optimize(self, y, z):
        #print("y: \n", y)
        #print("z: \n", z)
        #print("L: \n", L)
        L = self.L
        M = self.M
        Q = self.Q
        R2 = self.R2
        #u = self.matrix @ (y + L @ z)
        u = self.matrix @ (self.M.T @ self.Q @ (y + L @ z) + self.R @ self.eta)

        return u

    def vel_array_cb(self, msg):
        self.current_vs = msg.data

    def speed_cb(self, msg: Float32):
        self.current_v = msg.data
        self.current_v_lat = self.current_v * np.sin(np.arctan(np.tan(np.deg2rad(self.current_steer_angle)) * self.l_F / self.l_R))
        print("lat_v", self.current_v_lat)
        print("long_v", self.current_v)
    def wheel_odom_cb(self, msg):
        self.current_yaw_rate = msg.twist.twist.angular.z

    def spin_mon_cb(self, msg):
        self.spin_mon = msg.data

    def path_cb(self, msg: Path):
        self.current_path.clear()
        for pose in msg.poses:
            position = pose.pose.position
            self.current_path.append([position.x, position.y])
        self.kdtree = cKDTree(self.current_path)

    def find_closest_point(self, robot_state):
        # Query the closest point to the robot's position
        # print(robot_state)
        _, index = self.kdtree.query([robot_state[0], robot_state[1]])
        
        closest_point = self.current_path[index]
        
        return closest_point, index

    def normalize_angle(self, yaw):
        while yaw > np.pi / 2:
            yaw -= np.pi
        while yaw < -np.pi / 2:
            yaw += np.pi
        return yaw
        
    def calculate_path_heading(self, closest_idx):
        # Calculate the heading based on the closest point and its neighbors
        if closest_idx == 0:
            next_point = self.current_path[closest_idx + 1]
            closest_point = self.current_path[closest_idx]
        elif closest_idx == len(self.current_path) - 1:
            next_point = self.current_path[closest_idx]
            closest_point = self.current_path[closest_idx - 1]
        else:
            next_point = self.current_path[closest_idx + 1]
            closest_point = self.current_path[closest_idx - 1]
            
        delta_x = next_point[0] - closest_point[0]
        delta_y = next_point[1] - closest_point[1]
        
        theta_path = np.arctan2(delta_y, delta_x)
        
        return self.normalize_angle(theta_path)


    
    def get_error(self, x, y, yaw):
        robot_state = [x, y, yaw]
        closest_point, closest_idx = self.find_closest_point(robot_state)
        theta_path = self.calculate_path_heading(closest_idx)
        e_theta = self.normalize_angle(robot_state[2] - theta_path)
        e_y = (robot_state[1] - closest_point[1]) * np.cos(e_theta) - (robot_state[0] - closest_point[0]) * np.sin(e_theta)

        if e_y > 2.0:
            #print(e_y, e_theta)
            pass
        print("e_y: ", e_y)
        print("e_theta: ", e_theta)
        
        return e_y, e_theta, self.current_vs[closest_idx]
    
    def save_data(self):
        self.states.append([self.ys[0], self.ys[1]])
        self.vels.append([self.current_v, self.current_yaw_rate])
        self.inputs.append(self.eta[0])
        self.yaw_errors.append(self.spin_mon)
        
        
    def update(self):
        if self.kdtree is None:
            return
        if len(self.current_vs) == 0:
            return
        v_long = 0.5 * (self.current_v + self.current_vs[0])
        v_lat = self.current_v_lat
        v = np.sqrt(v_long**2 + v_lat**2)
        steer_angle = np.deg2rad(self.current_steer_angle)
        zs = np.zeros(self.horizon*2)
        qs = [[0, 0, 0]]
        
        e_l, init_e_h, _ = self.get_error(0, 0, 0)
        init_e_h = init_e_h + self.normalize_angle(np.arctan(v_lat / v_long))
        e_h = init_e_h
        y_0_l = e_l
        y_0_ldot = v * np.sin(e_h)
        dz = np.array([(y_0_l - self.ys[0]), (y_0_ldot - self.ys[1])])
        self.ys[0] = e_l
        self.ys[1] = y_0_ldot
        
        
        path_msg = Path()
        path_msg.header.frame_id = 'vehicle'
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        path_msg.poses.append(pose)
        for i in range(self.horizon-1):
            print("i: ", i)
            w = self.eta[i] / (v * np.cos(e_h))
            print("w:", w)
            new_state, v_lat = self.bicycle_model(v_long, v_lat, w, qs[-1], steer_angle)
            # print("new_state: ", new_state)
            
            qs.append(new_state)
            qs[-1][2] = self.normalize_angle(qs[-1][2])
            pose = PoseStamped()
            pose.pose.position.x = qs[-1][0]
            pose.pose.position.y = qs[-1][1]
            path_msg.poses.append(pose)
            e_l, e_h, v_long = self.get_error(*qs[i+1])
            w = np.clip(self.eta[i] / (v_long * np.cos(e_h)), -100.0, 100.0)
            steer_angle = np.rad2deg(2.9718 * w / (max(v_long, 1)))
            self.ys[i*2+2] = e_l
            self.ys[i*2+3] = v * np.sin(e_h)
            zs[i*2+2] = self.ys[i*2+2] - self.ys[(i-1)*2+2]
            zs[i*2+3] = self.ys[i*2+3] - self.ys[(i-1)*2+3]
        self.path_pub.publish(path_msg)
        #print(self.eta)
        #print(self.optimize(self.ys, dz))
        self.eta += self.optimize(self.ys, dz)
        #print("Eta: ", self.eta)
        #self.eta[0] += prev_u
        self.eta = np.clip(self.eta, -100, 100)
        w = np.clip(self.eta[0] / (self.current_v * np.cos(init_e_h)), -100.0, 100.0)
        
        msg = Float32()
        msg.data = w
        self.desired_w_pub.publish(msg)
        
        self.current_steer_angle = np.rad2deg(2.9718 * w / (max(self.current_vs[0], 1)))
        msg = Float32()
        msg.data = self.current_steer_angle
        self.publisher_steer_cmd.publish(msg)
        
            
            
        
'''
    def update(self):
        if self.kdtree is None:
            return
        if len(self.current_vs) == 0:
            return

        v = 0.5 * (self.current_v + self.current_vs[0])
        zs = np.zeros(self.horizon*2)
        qs = [[0, 0, 0]]

        e_l, init_e_h, _ = self.get_error(0, 0, 0)
        e_h = init_e_h
        y_0_l = e_l
        y_0_ldot = v * np.sin(e_h)
        dz = np.array([(y_0_l - self.ys[0]), (y_0_ldot - self.ys[1])])
        self.ys[0] = e_l
        self.ys[1] = y_0_ldot

        self.save_data()

        path_msg = Path()
        path_msg.header.frame_id = 'vehicle'
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        path_msg.poses.append(pose)

        for i in range(self.horizon-1):
            yaw = qs[i][-1]
            w = self.eta[i] / (v * np.cos(e_h))
            #print("w:", w)
            qs.append(self.rk4_step(v, w, qs[-1]))
            qs[-1][2] = self.normalize_angle(qs[-1][2])
            pose = PoseStamped()
            pose.pose.position.x = qs[-1][0]
            pose.pose.position.y = qs[-1][1]
            path_msg.poses.append(pose)
            e_l, e_h, v = self.get_error(*qs[i+1])
            #print(e_l, e_h)
            self.ys[i*2+2] = e_l
            self.ys[i*2+3] = v * np.sin(e_h)
            zs[i*2+2] = self.ys[i*2+2] - self.ys[(i-1)*2+2]
            zs[i*2+3] = self.ys[i*2+3] - self.ys[(i-1)*2+3]
        self.path_pub.publish(path_msg)
        #print(self.eta)
        #print(self.optimize(self.ys, dz))
        self.eta += self.optimize(self.ys, dz)
        #print("Eta: ", self.eta)
        #self.eta[0] += prev_u
        self.eta = np.clip(self.eta, -100, 100)
        w = np.clip(self.eta[0] / (self.current_v * np.cos(init_e_h)), -100.0, 100.0)

        msg = Float32()
        msg.data = w
        self.desired_w_pub.publish(msg)

        self.steer_angle = np.rad2deg(2.9718 * w / (max(self.current_vs[0], 1)))
        msg = Float32()
        msg.data = self.steer_angle
        self.publisher_steer_cmd.publish(msg)
'''



def main(args=None):
    rclpy.init(args=args)
    mpc = MPC(0.2, 25)

    try:
        rclpy.spin(mpc)
    except KeyboardInterrupt:
        with open('data.csv', 'w') as f:
            write = csv.writer(f)
            write.writerow(['el', 'el_dot', 'Lin Vel', 'Ang. Vel', 'Yaw Rate Error', 'Input'])
            for i in range(len(mpc.states)):
                row = mpc.states[i] + mpc.vels[i] + [mpc.yaw_errors[i]] + [mpc.inputs[i]] 
                write.writerow(row)
    finally:
        mpc.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()