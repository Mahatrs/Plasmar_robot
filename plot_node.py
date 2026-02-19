#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib
from std_msgs.msg import Float64, Float64MultiArray
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
import math
import threading
import time
import numpy as np
import os

class PlasmarPlotter(Node):
    def __init__(self):
        super().__init__('plasmar_plotter')
        
       # Données BlueRobotics pour la Lookup Table
        self.force_lut = np.array([
            -39.91, -39.72, -39.42, -38.83, -38.25, -37.95, -37.46, -37.27, -36.77,
            -36.38, -35.89, -35.21, -34.52, -33.83, -33.34, -32.46, -31.87, -31.38,
            -30.5 , -29.81, -29.32, -28.83, -28.05, -27.65, -27.16, -26.58, -26.09,
            -25.3 , -25.01, -24.61, -24.03, -23.34, -23.05, -22.36, -21.97, -21.57,
            -20.79, -20.4 , -19.81, -19.42, -19.02, -18.24, -17.75, -17.26, -16.67,
            -16.38, -15.79, -15.3 , -14.81, -14.61, -14.12, -13.73, -13.24, -12.75,
            -12.36, -11.77, -11.38, -10.98, -10.79, -10.3 , -10.  ,  -9.61,  -9.22,
             -8.83,  -8.53,  -8.04,  -7.65,  -7.26,  -7.06,  -6.67,  -6.37,  -6.08,
             -5.69,  -5.3 ,  -5.  ,  -4.71,  -4.31,  -4.12,  -3.82,  -3.43,  -3.14,
             -2.84,  -2.55,  -2.35,  -2.06,  -1.77,  -1.47,  -1.27,  -0.98,  -0.88,
             -0.69,  -0.49,  -0.39,   0.  ,   0.  ,   0.  ,   0.  ,   0.  ,   0.  ,
              0.  ,   0.  ,   0.  ,   0.  ,   0.  ,   0.  ,   0.  ,   0.  ,   0.  ,
              0.39,   0.49,   0.78,   0.98,   1.27,   1.47,   1.77,   2.16,   2.45,
              2.84,   3.14,   3.53,   3.92,   4.31,   4.61,   5.  ,   5.49,   5.88,
              6.28,   6.67,   7.06,   7.65,   8.04,   8.53,   8.92,   9.32,   9.71,
             10.1 ,  10.79,  11.18,  11.57,  12.16,  12.55,  13.04,  13.63,  14.12,
             14.51,  15.1 ,  15.59,  16.18,  16.57,  17.26,  17.85,  18.44,  18.93,
             19.52,  20.1 ,  20.79,  21.38,  21.77,  22.36,  23.34,  23.83,  24.71,
             25.3 ,  25.99,  26.77,  27.07,  27.85,  28.34,  29.22,  29.91,  30.5 ,
             31.09,  31.58,  32.36,  33.05,  33.54,  34.42,  35.4 ,  36.09,  36.68,
             37.46,  38.15,  38.83,  39.81,  40.7 ,  41.68,  42.17,  42.95,  44.23,
             44.42,  45.6 ,  46.19,  46.97,  47.46,  48.35,  49.13,  49.82,  50.41,
             50.8 ,  51.19,  51.48
        ])
        
        self.rpm_lut = np.array([
            -3465, -3468, -3449, -3421, -3410, -3395, -3374, -3356, -3323, -3307,
            -3282, -3248, -3223, -3207, -3177, -3153, -3111, -3086, -3061, -3050,
            -3003, -2988, -2952, -2918, -2896, -2865, -2845, -2821, -2792, -2759,
            -2736, -2712, -2681, -2661, -2629, -2599, -2576, -2548, -2517, -2482,
            -2440, -2419, -2369, -2345, -2306, -2278, -2248, -2222, -2199, -2157,
            -2140, -2106, -2081, -2040, -2001, -1968, -1936, -1903, -1870, -1840,
            -1807, -1773, -1738, -1705, -1667, -1630, -1589, -1554, -1519, -1482,
            -1448, -1410, -1362, -1325, -1285, -1246, -1199, -1153, -1113, -1064,
            -1016,  -970,  -916,  -869,  -810,  -758,  -697,  -641,  -578,  -513,
             -450,  -388,  -317,     0,     0,     0,     0,     0,     0,     0,
                0,     0,     0,     0,     0,     0,     0,     0,   308,   375,
              441,   508,   573,   630,   688,   749,   804,   859,   906,   962,
             1007,  1057,  1104,  1148,  1195,  1237,  1279,  1319,  1359,  1409,
             1444,  1480,  1516,  1549,  1583,  1629,  1668,  1699,  1740,  1766,
             1802,  1835,  1865,  1901,  1930,  1961,  1994,  2031,  2071,  2106,
             2129,  2160,  2190,  2219,  2244,  2266,  2305,  2342,  2392,  2416,
             2446,  2475,  2498,  2538,  2584,  2609,  2639,  2663,  2688,  2711,
             2735,  2766,  2809,  2822,  2854,  2888,  2903,  2934,  2973,  3002,
             3042,  3065,  3108,  3125,  3153,  3180,  3204,  3242,  3260,  3299,
             3308,  3351,  3379,  3406,  3428,  3438,  3455,  3494,  3516,  3527,
             3533
        ])
        
        self.reset_data()
        
        # Subscribers
        self.create_subscription(Odometry, '/plasmar/odom', self.odom_callback, 100)
        self.create_subscription(Twist, '/plasmar/applied_forces', self.forces_callback, 100)
        self.create_subscription(Float64, '/plasmar/ring_angle', self.ring_callback, 100)
        self.create_subscription(Float64MultiArray, '/plasmar/thrusters', self.thrusters_callback, 100)
        
        # Threads
        self.plot_thread = threading.Thread(target=self.plot_loop)
        self.plot_thread.daemon = True
        self.plot_thread.start()
        
        self.get_logger().info("Plotter Node Prêt ")

    def reset_data(self):
        self.get_logger().warn(">>> DETECTION RESET SIMULATION : Effacement des graphiques.")
        self.start_time = self.get_clock().now()
        self.times = []
        self.roll, self.pitch, self.yaw = [], [], []
        self.x_real, self.y_real, self.z_real = [], [], []
        self.fx, self.fy, self.fz, self.mz = [], [], [], []
        self.m1, self.m2, self.m3, self.m4 = [], [], [], []
        self.ring_angle_deg = []
        self.has_moved_far = False

    def get_euler(self, q):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.pi / 2 if abs(sinp) >= 1 else math.asin(sinp)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        dist_from_origin = math.sqrt(x**2 + y**2 + z**2)
        if dist_from_origin > 1.0:
            self.has_moved_far = True
        if self.has_moved_far and dist_from_origin < 0.1:
            self.reset_data()
            return

        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.times.append(t)
        self.x_real.append(x)
        self.y_real.append(y)
        self.z_real.append(z)
        
        r, p, y_ang = self.get_euler(msg.pose.pose.orientation)
        self.roll.append(r)
        self.pitch.append(p)
        self.yaw.append(y_ang)

    def forces_callback(self, msg):
        if len(self.times) > 0:
            self.fx.append(msg.linear.x)
            self.fy.append(msg.linear.y)
            self.fz.append(msg.linear.z)
            self.mz.append(msg.angular.z)
            
    def ring_callback(self, msg):
        deg = math.degrees(msg.data)
        self.ring_angle_deg.append(deg)
            
    def thrusters_callback(self, msg):
        self.m1.append(msg.data[0])
        self.m2.append(msg.data[1])
        self.m3.append(msg.data[2])
        self.m4.append(msg.data[3])

    def plot_loop(self):
        while rclpy.ok():
            time.sleep(1.0)
            try:
                self.save_plots()
            except Exception as e:
                self.get_logger().error(f"Erreur plotting: {e}")

    def save_plots(self):
        t_all = list(self.times)
        if len(t_all) < 5: return

        limit = 6000
        len_odom = min(len(t_all), len(self.x_real))
        start_idx = max(0, len_odom - limit)
        
        t = t_all[start_idx:len_odom]
        x = list(self.x_real)[start_idx:len_odom]
        y = list(self.y_real)[start_idx:len_odom]
        z = list(self.z_real)[start_idx:len_odom]
        
        roll = list(self.roll)[start_idx:len_odom]
        pitch = list(self.pitch)[start_idx:len_odom]
        
        angle_ring = list(self.ring_angle_deg)[-limit:]
        
        def get_last_n(data_list, n):
            d = list(data_list)
            if len(d) >= n: return d[-n:]
            return [0.0]*(n-len(d)) + d

        m1_vals = get_last_n(self.m1, len(t))
        m2_vals = get_last_n(self.m2, len(t))
        m3_vals = get_last_n(self.m3, len(t))
        m4_vals = get_last_n(self.m4, len(t))

        # Interpolation
        m1_rpm = np.interp(m1_vals, self.force_lut, self.rpm_lut)
        m2_rpm = np.interp(m2_vals, self.force_lut, self.rpm_lut)
        m3_rpm = np.interp(m3_vals, self.force_lut, self.rpm_lut)
        m4_rpm = np.interp(m4_vals, self.force_lut, self.rpm_lut)

        if len(t) > 0:
            common_xlim = [t[0], t[-1]]
        else:
            common_xlim = [0, 1]

        # Tracés 2D
        fig, axs = plt.subplots(4, 2, figsize=(16, 18))
        fig.suptitle(f"Analyse Mission PlaSMAR (Derniers {len(t)} pts)")

        axs[0, 0].plot(x, z, 'b-', linewidth=2, label='Trajectoire Réelle')
        axs[0, 0].set_title("Vue de Profil (X/Z)")
        axs[0, 0].set_ylabel("Profondeur Z (m)")
        axs[0, 0].set_xlabel("Distance X (m)")
        axs[0, 0].grid(True, linestyle='--', alpha=0.7)
        axs[0, 0].set_aspect('equal', 'datalim')
        axs[0, 0].legend()

        axs[0, 1].plot(x, y, 'r-o', linewidth=2, label='Trajectoire Réelle')
        axs[0, 1].set_title("Vue de Dessus (XY)")
        axs[0, 1].axis('equal')
        axs[0, 1].grid(True, linestyle='--', alpha=0.7)
        axs[0, 1].legend()

        axs[1, 0].plot(t, roll, 'r', label='Roll')
        axs[1, 0].plot(t, pitch, 'g', label='Pitch')
        axs[1, 0].set_title("Stabilité Angulaire")
        axs[1, 0].grid(True)
        axs[1, 0].set_xlim(common_xlim)
        axs[1, 0].legend()
        
        t_ring = np.linspace(t[0], t[-1], len(angle_ring)) if len(angle_ring) > 1 and len(t) > 1 else range(len(angle_ring))
        axs[1, 1].plot(t_ring, angle_ring, 'purple', linewidth=2, label='Angle Anneau')
        axs[1, 1].set_title("Orientation Propulseurs")
        axs[1, 1].set_ylabel("Alpha (deg)")
        axs[1, 1].set_yticks(range(-180, 181, 45))
        axs[1, 1].grid(True)

        axs[2, 0].plot(t, m1_rpm, label='M1 (G)', color='blue')
        axs[2, 0].plot(t, m2_rpm, label='M2 (D)', color='cyan', linestyle='--')
        axs[2, 0].set_title("Vitesse Rotation Arrières (RPM) - Modèle T200")
        axs[2, 0].set_ylabel("RPM")
        axs[2, 0].grid(True)
        axs[2, 0].set_xlim(common_xlim)
        axs[2, 0].legend()

        axs[2, 1].plot(t, m3_rpm, label='M3 (Anneau)', color='orange')
        axs[2, 1].plot(t, m4_rpm, label='M4 (Anneau)', color='red', linestyle='--')
        axs[2, 1].set_title("Vitesse Rotation Anneau (RPM) - Modèle T200")
        axs[2, 1].set_ylabel("RPM")
        axs[2, 1].grid(True)
        axs[2, 1].set_xlim(common_xlim)
        axs[2, 1].legend()

        axs[3, 0].plot(t, m1_vals, label='Force M1', color='blue')
        axs[3, 0].plot(t, m2_vals, label='Force M2', color='cyan', linestyle='--')
        axs[3, 0].set_title("Effort de Commande Arrière (Trapèzes de Newton)")
        axs[3, 0].set_ylabel("Force (N)")
        axs[3, 0].set_xlabel("Temps (s)")
        axs[3, 0].grid(True)
        axs[3, 0].set_xlim(common_xlim)
        axs[3, 0].legend()

        axs[3, 1].plot(t, m3_vals, label='Force M3', color='orange')
        axs[3, 1].plot(t, m4_vals, label='Force M4', color='red', linestyle='--')
        axs[3, 1].set_title("Effort de Commande Anneau (Trapèzes de Newton)")
        axs[3, 1].set_ylabel("Force (N)")
        axs[3, 1].set_xlabel("Temps (s)")
        axs[3, 1].grid(True)
        axs[3, 1].set_xlim(common_xlim)
        axs[3, 1].legend()

        # Écriture atomique pour garder l'image dans docker avec la commande cp
        temp_filename = 'simulation_result_tmp.png'
        final_filename = 'simulation_result.png'
        plt.tight_layout()
        plt.savefig(temp_filename)
        plt.close(fig)
        os.replace(temp_filename, final_filename)

        # Tracés 3D
        fig3d = plt.figure(figsize=(10, 8))
        ax = fig3d.add_subplot(111, projection='3d')
        
        x_full = list(self.x_real)
        y_full = list(self.y_real)
        z_full = list(self.z_real)
        
        if len(x_full) > 1:
            ax.plot(x_full, y_full, z_full, linewidth=2, label='Trajet Complet')
            ax.scatter(x_full[0], y_full[0], z_full[0], c='g', marker='o', s=50, label='Start')
            ax.scatter(x_full[-1], y_full[-1], z_full[-1], c='r', marker='x', s=50, label='End')
            
            max_range = np.array([np.ptp(x_full), np.ptp(y_full), np.ptp(z_full)]).max() / 2.0
            mid_x = (np.max(x_full) + np.min(x_full)) * 0.5
            mid_y = (np.max(y_full) + np.min(y_full)) * 0.5
            mid_z = (np.max(z_full) + np.min(z_full)) * 0.5
            
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
            
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("Z (m)")
            ax.set_title("Visualisation 3D")
            ax.legend()

            temp_3d = 'simulation_3d_tmp.png'
            final_3d = 'simulation_3d.png'
            plt.savefig(temp_3d)
            plt.close(fig3d)
            os.replace(temp_3d, final_3d)

def main(args=None):
    rclpy.init(args=args)
    node = PlasmarPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
