#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import numpy as np
import math

from plasmar_sim.plasmar_maths import PlasmarModel

class PlasmarTrajectoryNode(Node):
    def __init__(self):
        super().__init__('traj_tracking_node')

        self.mission_active = False
        self.model = PlasmarModel()

        # Vitesses de référence (m/s)
        self.dive_speed = 0.4
        self.scan_speed = 1.0
        self.approach_speed = 1.0
        
        # Vitesse latérale douce (0.2 m/s) pour laisser de la puissance au contrôle de profondeur
        self.lateral_speed = 0.1
        
        # Etat
        self.eta = np.zeros(6)
        self.nu = np.zeros(6)

        # Cible
        self.eta_d = np.zeros(6)
        self.eta_dot_d = np.zeros(6)
        self.nu_d_prev = np.zeros(6)

        # Trajectoire
        self.waypoints = []
        self.current_wp_index = 0
        self.segment_start = None
        self.segment_end = None
        self.segment_type = 'NORMAL'
        self.segment_vec = None
        self.segment_len = 0.0
        self.dist_on_segment = 0.0
        
        self.locked_yaw = 0.0

        # Gains
        self.Kp = np.diag([1.2, 1.2, 1.2, 0.8, 0.8, 1.5])
        self.Ki = np.diag([0.1, 0.1, 0.1, 0.0, 0.0, 0.05])
        self.integral_error = np.zeros(6)
        self.Kv = np.diag([8.0, 8.0, 8.0, 4.0, 4.0, 10.0])

        # ROS
        self.path_pub = self.create_publisher(Path, '/plasmar/desired_path', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/plasmar/target', 10)
        self.auto_cmd_pub = self.create_publisher(Twist, '/plasmar/auto_cmd_vel', 10)
        
        self.create_subscription(Bool, '/plasmar/auto_trigger', self.mission_control_callback, 10)
        self.create_subscription(Odometry, '/plasmar/odom', self.odom_callback, 10)

        self.create_timer(1.0, self.publish_path)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Noeud de Suivi de Trajectoire Initialisé")

    def odom_callback(self, msg):
        self.eta[0] = msg.pose.pose.position.x
        self.eta[1] = msg.pose.pose.position.y
        self.eta[2] = msg.pose.pose.position.z
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.eta[5] = math.atan2(siny_cosp, cosy_cosp)
        
        self.nu[0] = msg.twist.twist.linear.x
        self.nu[1] = msg.twist.twist.linear.y
        self.nu[2] = msg.twist.twist.linear.z
        self.nu[5] = msg.twist.twist.angular.z

    def mission_control_callback(self, msg):
        if msg.data and not self.mission_active:
            self.get_logger().info("Mission Autonome : Démarrage")
            self.integral_error = np.zeros(6)
            self.build_mission_path()
            self.mission_active = True
        elif not msg.data:
            self.mission_active = False
            self.get_logger().info("Mission Autonome : Arrêt")
            self.integral_error = np.zeros(6)
            self.auto_cmd_pub.publish(Twist())

    def build_mission_path(self):
        current_pos = self.eta[0:3].copy()
        z_mission = -1.5
        
        dive_wp = (np.array([current_pos[0], current_pos[1], z_mission]), 'NORMAL')
        start_wp = (current_pos, 'NORMAL')
        scan_wps = self.generate_seabed_scan(Lx=20.0, Ly=15.0, n_strips=5, z0=z_mission)
        
        self.waypoints = [start_wp, dive_wp] + scan_wps
        self.current_wp_index = 0
        self.setup_segment(0)
        
        self.eta_d = self.eta.copy()
        self.dist_on_segment = 0.0

    def generate_seabed_scan(self, Lx=10.0, Ly=10.0, n_strips=3, z0=0.0):
        xs = np.linspace(0, Lx, n_strips)
        wps = []
        direction = 1
        
        wps.append((np.array([0.0, 0.0, z0]), 'NORMAL'))
        
        for i, x in enumerate(xs):
            y_target = Ly if direction == 1 else 0.0
            p_end_line = np.array([x, y_target, z0])
            wps.append((p_end_line, 'NORMAL'))
            
            if i < len(xs) - 1:
                x_next = xs[i+1]
                p_next_start = np.array([x_next, y_target, z0])
                
                if i <= 1:
                    type_transition = 'LATERAL'
                else:
                    type_transition = 'NORMAL'
                
                wps.append((p_next_start, type_transition))
            
            direction *= -1
            
        return wps

    def setup_segment(self, idx):
        if idx >= len(self.waypoints) - 1: return False
        
        p_start, type_start = self.waypoints[idx]
        p_end, type_end = self.waypoints[idx+1]
        
        self.segment_start = p_start
        self.segment_end = p_end
        self.segment_type = type_end
        
        vec = self.segment_end - self.segment_start
        self.segment_len = np.linalg.norm(vec)
        
        if self.segment_len > 0.001:
            self.segment_vec = vec / self.segment_len
        else:
            self.segment_vec = np.zeros(3)
        
        self.dist_on_segment = 0.0
        self.integral_error = np.zeros(6)
        
        if self.segment_type == 'NORMAL':
            if self.segment_len > 0.001:
                self.locked_yaw = math.atan2(self.segment_vec[1], self.segment_vec[0])
        elif self.segment_type == 'LATERAL':
            pass
            
        return True

    def update_virtual_target(self, dt):
        if self.segment_start is None: return False

        target_speed = self.scan_speed
        if self.current_wp_index == 0: target_speed = self.dive_speed
        elif self.current_wp_index == 1: target_speed = self.approach_speed
        elif self.segment_type == 'LATERAL': target_speed = self.lateral_speed

        dist_remaining = self.segment_len - self.dist_on_segment
        if dist_remaining < 0.4:
            factor = dist_remaining / 0.4
            target_speed = target_speed * factor
            if target_speed < 0.1: target_speed = 0.1

        self.dist_on_segment += target_speed * dt

        if self.dist_on_segment >= self.segment_len:
            self.current_wp_index += 1
            if not self.setup_segment(self.current_wp_index):
                return False
        
        self.eta_d[0:3] = self.segment_start + (self.segment_vec * self.dist_on_segment)
        self.eta_d[5] = self.locked_yaw
        
        self.eta_dot_d[0:3] = self.segment_vec * target_speed
        self.eta_dot_d[5] = 0.0
        
        return True

    def control_loop(self):
        if not self.mission_active: return
        dt = 0.05
        
        if not self.update_virtual_target(dt):
            self.mission_active = False
            self.auto_cmd_pub.publish(Twist())
            return

        self.publish_target_viz()

        # Loi de commande
        e_eta = self.eta_d - self.eta
        
        while e_eta[5] > math.pi: e_eta[5] -= 2*math.pi
        while e_eta[5] < -math.pi: e_eta[5] += 2*math.pi
        
        e_eta = np.clip(e_eta, -2.0, 2.0)
        self.integral_error += e_eta * dt
        self.integral_error = np.clip(self.integral_error, -5.0, 5.0)

        # Cinématique
        J_inv = self.model.get_J_inv(self.eta)
        terme_correctif = (self.Kp @ e_eta) + (self.Ki @ self.integral_error)
        nu_d = J_inv @ (self.eta_dot_d + terme_correctif)
        nu_d[1] = np.clip(nu_d[1], -1.5, 1.5)
         
        # Dynamique
        e_nu = nu_d - self.nu
        nu_dot_d = (nu_d - self.nu_d_prev) / dt
        self.nu_d_prev = nu_d
        M = self.model.get_M()
        C = self.model.get_C(self.nu)
        D = self.model.get_D(self.nu)
        g_vec = self.model.get_g(self.eta)
        
        acc_term = nu_dot_d + (self.Kv @ e_nu)
        tau = (M @ acc_term) + (C @ self.nu) + (D @ self.nu) - g_vec

        cmd = Twist()
        tau = np.clip(tau, -80.0, 80.0)
        
        cmd.linear.x, cmd.linear.y, cmd.linear.z = float(tau[0]), float(tau[1]), float(tau[2])
        cmd.angular.x, cmd.angular.y, cmd.angular.z = float(tau[3]), float(tau[4]), float(tau[5])
        
        self.auto_cmd_pub.publish(cmd)

    def publish_target_viz(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = "map"
        msg.pose.position.x = self.eta_d[0]
        msg.pose.position.y = self.eta_d[1]
        msg.pose.position.z = self.eta_d[2]
        cy = math.cos(self.eta_d[5] * 0.5); sy = math.sin(self.eta_d[5] * 0.5)
        cp = math.cos(0); sp = math.sin(0)
        cr = math.cos(0); sr = math.sin(0)
        msg.pose.orientation.w = cr*cp*cy + sr*sp*sy
        msg.pose.orientation.x = sr*cp*cy - cr*sp*sy
        msg.pose.orientation.y = cr*sp*cy + sr*cp*sy
        msg.pose.orientation.z = cr*cp*sy - sr*sp*cy
        self.target_pub.publish(msg)
        
    def publish_path(self):
        if not self.waypoints: return
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg(); path_msg.header.frame_id = "map"
        for item in self.waypoints:
            p = item[0]
            pose = PoseStamped(); pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(p[0]), float(p[1]), float(p[2])
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlasmarTrajectoryNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
