#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray # <--- AJOUT
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration
import math
import numpy as np

# Import de la librairie
from plasmar_sim.plasmar_maths import PlasmarModel

class PlasmarPhysicsSim(Node):
    """
    Simulateur de la dynamique du robot PlaSMAR.
    """
    
    def __init__(self):
        super().__init__('plasmar_dynamic_sim')
        self.br = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/plasmar/odom', 10)
        self.ring_pub = self.create_publisher(Float64, '/plasmar/ring_angle', 10)
        self.force_real_pub = self.create_publisher(Twist, '/plasmar/applied_forces', 10)
        self.thruster_pub = self.create_publisher(Float64MultiArray, '/plasmar/thrusters', 10)

        self.model = PlasmarModel()

        # Dynamique de l'anneau rotatif
        self.ring_speed_max = 2.0  # rad/s
        self.ring_acc = 3.0     # rad/s^2
        self.alpha_ring = 0.0
        self.alpha_cmd = 0.0
        self.ring_curr_vel = 0.0

        # Vecteurs d'état
        self.eta = np.zeros(6) # [x, y, z, phi, theta, psi]
        self.nu = np.zeros(6) # [u, v, w, p, q, r]
        # self.eta[3] = 0.785  # Roll initial
        # self.eta[4] = 0.785  # Pitch initial
        
        
        self.u_current = np.zeros(4) # [u1, u2, u3, u4]
        self.tau_applied = np.zeros(6)
        
        # Fréquence de simulation et modélisation des actionneurs
        self.dt = 0.02
        self.MAX_RATE_NEWTON = 100.0 * self.dt # 1er ordre
        self.MAX_FORCE_ABS = 40.0 # Saturation T200


        self.sim_time = self.get_clock().now()
        
        M = self.model.get_M()
        self.M_inv = np.linalg.inv(M)
        
        self.tau_target_vector = np.zeros(6)

        self.create_subscription(Twist, '/plasmar/cmd_vel', self.cmd_callback, 10)
        self.create_timer(self.dt, self.update_physics)
        
        self.get_logger().info("PlaSMAR Physics: Mode Hybride (Logique Etudiante + Matrice Prof)")

    # Commande de l'anneau
    def cmd_callback(self, msg):
        gain_force = 1.0
        
        Xc = msg.linear.x * gain_force
        Yc = msg.linear.y * gain_force
        Zc = msg.linear.z * gain_force
        Nc = msg.angular.z * gain_force

        # Calcul de l'angle
        raw_angle = 0.0
        if abs(Yc) > 0.1 or abs(Zc) > 0.1:
            raw_angle = math.atan2(Yc, -Zc)
    
        target_force = math.sqrt(Yc**2 + Zc**2)
        
        if abs(raw_angle) > (math.pi / 2.0 + 0.1):
            if raw_angle > 0:
                self.alpha_cmd = raw_angle - math.pi
            else:
                self.alpha_cmd = raw_angle + math.pi
            target_force = -target_force
        else:
            self.alpha_cmd = raw_angle

        
        Fy_cmd = target_force * math.sin(self.alpha_cmd)
        Fz_cmd = -target_force * math.cos(self.alpha_cmd)
        
        self.tau_target_vector = np.array([Xc, Fy_cmd, Fz_cmd, 0.0, 0.0, Nc])

    # Allocation propulseurs
    def get_motor_allocation(self, tau_d, alpha):
        R = 0.11
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        # Matrice B
        B = np.array([
            [1,  1,   0,   0],
            [0,  0,  sa,  sa],
            [0,  0, -ca, -ca],
            [0,  0,  -R,   R],
            [0,  0,   0,   0],
            [-R, R,   0,   0]
        ])
        return B, np.linalg.pinv(B) @ tau_d

    def update_physics(self):
        self.sim_time += Duration(seconds=self.dt)
        
        # Mise à jour anneau
        diff = self.alpha_cmd - self.alpha_ring
        while diff > math.pi: diff -= 2*math.pi
        while diff < -math.pi: diff += 2*math.pi
        
        target_vel = diff * 4.0
        target_vel = max(min(target_vel, self.ring_speed_max), -self.ring_speed_max)
        
        vel_diff = target_vel - self.ring_curr_vel
        max_vel_step = self.ring_acc * self.dt
        
        if abs(vel_diff) < max_vel_step:
            self.ring_curr_vel = target_vel
        else:
            self.ring_curr_vel += math.copysign(max_vel_step, vel_diff)
            
        self.alpha_ring += self.ring_curr_vel * self.dt
        
        # Allocation et propulseurs
        B_matrix, u_target = self.get_motor_allocation(self.tau_target_vector, self.alpha_ring)

        for i in range(4):
            error = u_target[i] - self.u_current[i]
            if error > self.MAX_RATE_NEWTON:
                self.u_current[i] += self.MAX_RATE_NEWTON
            elif error < -self.MAX_RATE_NEWTON:
                self.u_current[i] -= self.MAX_RATE_NEWTON
            else:
                self.u_current[i] = u_target[i]
            
            # Saturation
            if self.u_current[i] > self.MAX_FORCE_ABS: self.u_current[i] = self.MAX_FORCE_ABS
            elif self.u_current[i] < -self.MAX_FORCE_ABS: self.u_current[i] = -self.MAX_FORCE_ABS

        self.tau_applied = B_matrix @ self.u_current
        
        msg_thrust = Float64MultiArray()
        msg_thrust.data = self.u_current.tolist()
        self.thruster_pub.publish(msg_thrust)
        
        msg_forces = Twist()
        msg_forces.linear.x = self.tau_applied[0]
        msg_forces.linear.y = self.tau_applied[1]
        msg_forces.linear.z = self.tau_applied[2]
        msg_forces.angular.x = self.tau_applied[3]
        msg_forces.angular.z = self.tau_applied[5]
        self.force_real_pub.publish(msg_forces)

        # Stabilité passive
        theta = self.eta[4]; q = self.nu[4]
        self.tau_applied[4] += -(40.0 * theta) - (15.0 * q)
        
        
        C = self.model.get_C(self.nu)
        D = self.model.get_D(self.nu)
        g_vec = self.model.get_g(self.eta)
        
        hydro_forces = (C @ self.nu) + (D @ self.nu)
        total_forces = self.tau_applied - hydro_forces + g_vec
        
        # Intégration Euler
        acc = self.M_inv @ total_forces
        self.nu += acc * self.dt

        phi, theta, psi = self.eta[3:6]
        cpsi, spsi = math.cos(psi), math.sin(psi)
        ct, st = math.cos(theta), math.sin(theta)
        cp, sp = math.cos(phi), math.sin(phi)

        R_mat = np.array([
            [cpsi*ct, -spsi*cp + cpsi*st*sp, spsi*sp + cpsi*st*cp],
            [spsi*ct, cpsi*cp + spsi*st*sp, -cpsi*sp + spsi*st*cp],
            [-st,     ct*sp,                 ct*cp]
        ])
        
        tt = math.tan(theta)
        T_ang = np.array([
            [1, sp*tt, cp*tt],
            [0, cp, -sp],
            [0, sp/ct, cp/ct]
        ])

        self.eta[0:3] += (R_mat @ self.nu[0:3]) * self.dt
        self.eta[3:6] += (T_ang @ self.nu[3:6]) * self.dt

        self.publish_state()

    def publish_state(self):
        current_time = self.sim_time.to_msg()
        # current_time = self.get_clock().now().to_msg()
        msg_angle = Float64(); msg_angle.data = self.alpha_ring; self.ring_pub.publish(msg_angle)
        
        t = TransformStamped()
        t.header.stamp = current_time; t.header.frame_id = "map"; t.child_frame_id = "base_link"
        t.transform.translation.x = self.eta[0]; t.transform.translation.y = self.eta[1]; t.transform.translation.z = self.eta[2]
        
        cy = math.cos(self.eta[5]*0.5); sy = math.sin(self.eta[5]*0.5)
        cp = math.cos(self.eta[4]*0.5); sp = math.sin(self.eta[4]*0.5)
        cr = math.cos(self.eta[3]*0.5); sr = math.sin(self.eta[3]*0.5)
        t.transform.rotation.w = cr*cp*cy + sr*sp*sy; t.transform.rotation.x = sr*cp*cy - cr*sp*sy
        t.transform.rotation.y = cr*sp*cy + sr*cp*sy; t.transform.rotation.z = cr*cp*sy - sr*sp*cy
        # self.br.sendTransform(t)
        
        t_ring = TransformStamped()
        t_ring.header.stamp = current_time; t_ring.header.frame_id = "base_link"; t_ring.child_frame_id = "ring_link"
        t_ring.transform.rotation.x = math.sin(self.alpha_ring/2.0); t_ring.transform.rotation.w = math.cos(self.alpha_ring/2.0)
        self.br.sendTransform([t, t_ring])
        # self.br.sendTransform(t_ring)
        
        odom = Odometry()
        odom.header.stamp = current_time; odom.header.frame_id = "map"; odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.eta[0]; odom.pose.pose.position.y = self.eta[1]; odom.pose.pose.position.z = self.eta[2]
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = self.nu[0]; odom.twist.twist.linear.y = self.nu[1]; odom.twist.twist.linear.z = self.nu[2]
        odom.twist.twist.angular.x = self.nu[3]; odom.twist.twist.angular.y = self.nu[4]; odom.twist.twist.angular.z = self.nu[5]
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = PlasmarPhysicsSim()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
