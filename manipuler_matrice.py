#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mavros_msgs.msg import State, OverrideRCIn


def clamp(x, lo, hi): #Fonction de saturation
    return max(lo, min(hi, x)) 


class RCAllocationB(Node): #Noeud allocation d'actionneurs PlaSMAR via la matrice B
    

    def __init__(self):
        super().__init__("rc_allocation_B") 

        self.declare_parameter("rate_hz", 30.0)        # Fréquence d’envoi des commandes
        self.declare_parameter("deadband", 0.03)       # zone morte sur les commandes

        self.declare_parameter("ch_rear_left", 1)      # thruster arrière gauche
        self.declare_parameter("ch_rear_right", 2)     # thruster arrière droit
        self.declare_parameter("ch_front_left", 3)     # thruster avant gauche (anneau)
        self.declare_parameter("ch_front_right", 4)    # thruster avant droit (anneau)

        # Rayon de la coque utilisé dans la matrice B
        self.declare_parameter("R_hull", 0.11)          # distance thruster–centre

        # Gains reliant Twist → efforts
        self.declare_parameter("k_X", 30.0)             # avance/recul
        self.declare_parameter("k_Y", 30.0)             # déplacement latéral
        self.declare_parameter("k_Z", 30.0)             # montée/descente
        self.declare_parameter("k_N", 5.0)              # rotation en lacet

        # Conversion effort en PWM
        self.declare_parameter("pwm_center", 1500)      # neutre PWM
        self.declare_parameter("pwm_per_newton", 8.0)   # pente N en µs
        self.declare_parameter("pwm_min", 1100)         # limite basse PWM
        self.declare_parameter("pwm_max", 1900)         # limite haute PWM

        self.declare_parameter("u_max_abs", 40.0)       # effort max admissible

        # Limitation de variation
        self.declare_parameter("use_rate_limiter", True)   # active le lissage
        self.declare_parameter("du_max_per_s", 120.0)      # variation max par seconde

        # Lecture paramètres
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.deadband = float(self.get_parameter("deadband").value)

        self.ch_rl = int(self.get_parameter("ch_rear_left").value)
        self.ch_rr = int(self.get_parameter("ch_rear_right").value)
        self.ch_fl = int(self.get_parameter("ch_front_left").value)
        self.ch_fr = int(self.get_parameter("ch_front_right").value)

        self.R = float(self.get_parameter("R_hull").value)

        self.kX = float(self.get_parameter("k_X").value)
        self.kY = float(self.get_parameter("k_Y").value)
        self.kZ = float(self.get_parameter("k_Z").value)
        self.kN = float(self.get_parameter("k_N").value)

        self.pwm_center = int(self.get_parameter("pwm_center").value)
        self.pwm_per_N = float(self.get_parameter("pwm_per_newton").value)
        self.pwm_min = int(self.get_parameter("pwm_min").value)
        self.pwm_max = int(self.get_parameter("pwm_max").value)

        self.u_max_abs = float(self.get_parameter("u_max_abs").value)

        self.use_rl = bool(self.get_parameter("use_rate_limiter").value)
        self.du_max_per_s = float(self.get_parameter("du_max_per_s").value)

        self.dt = 1.0 / self.rate_hz                 # pas de temps
        self.du_max_step = self.du_max_per_s * self.dt  # variation max par itération

       
        self.connected = False                       # vrai si MAVROS connecté
        self.last_cmd = Twist()                      # dernière commande clavier
        self.u_prev = np.zeros(4)                    # effort précédent (lissage)


        self.pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)  # Publication RC override
        self.create_subscription(State, '/mavros/state', self.on_state, 10)       # Abonnement état MAVROS
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)          # Abonnement commandes clavier

    def on_state(self, msg: State):
        self.connected = bool(msg.connected)        # Vrai uniquement si MAVROS est bien connecté au FCU

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg                          # mémorise la commande reçue

    def build_B(self, alpha: float) -> np.ndarray:
        sa = math.sin(alpha)                         # sin(angle anneau)
        ca = math.cos(alpha)                         # cos(angle anneau)
        R = self.R                                   # rayon de la coque
        return np.array([                            # matrice d’allocation B
            [1.0, 1.0, 0.0, 0.0],                    # surge
            [0.0, 0.0, sa,  sa ],                    # sway
            [0.0, 0.0, -ca, -ca],                    # heave
            [0.0, 0.0, -R,  R ],                     # roll
            [0.0, 0.0, 0.0, 0.0],                    # pitch non utilisé
            [-R,  R,  0.0, 0.0],                     # yaw
        ], dtype=float)

    def u_to_pwm(self, uN: float) -> int:
        pwm = int(self.pwm_center + self.pwm_per_N * uN)  # effort en PWM
        return int(clamp(pwm, self.pwm_min, self.pwm_max))  # saturation PWM

    def set_ch(self, channels, ch_1based: int, pwm: int):
        idx = ch_1based - 1                           
        if 0 <= idx < 18:
            channels[idx] = int(pwm)                  

    def on_timer(self):
        if not self.connected:
            return                                    # rien si non connecté

        vx = self.last_cmd.linear.x                   # avance/recul
        vy = self.last_cmd.linear.y                   # déplacement latéral
        vz = self.last_cmd.linear.z                   # montée/descente
        wz = self.last_cmd.angular.z                  # rotation yaw

        if abs(vx) < self.deadband: vx = 0.0          # zone morte
        if abs(vy) < self.deadband: vy = 0.0
        if abs(vz) < self.deadband: vz = 0.0
        if abs(wz) < self.deadband: wz = 0.0

        Xc = self.kX * vx                             # effort de surge
        Yc = self.kY * vy                             # effort de sway
        Zc = self.kZ * vz                             # effort de heave
        Nc = self.kN * wz                             # moment de yaw

        Kc = 0.0                                     # pas de contrôle du roulis ici

        tau_c = np.array([Xc, Yc, Zc, Kc, 0.0, Nc])   # vecteur d’effort global

        if abs(Yc) < 1e-6 and abs(Zc) < 1e-6:
            alpha_c = 0.0                             # anneau neutre
        else:
            alpha_c = math.atan2(Yc, -Zc)             # orientation de l’anneau

        B = self.build_B(alpha_c)                     # matrice B locale
        u = np.linalg.pinv(B) @ tau_c                 # allocation par pseudo-inverse

        u = np.clip(u, -self.u_max_abs, self.u_max_abs)  # limite moteur

        if self.use_rl:
            du = u - self.u_prev                      # variation d’effort
            du = np.clip(du, -self.du_max_step, self.du_max_step)
            u = self.u_prev + du
            self.u_prev = u                           # mémorisation

        rc = OverrideRCIn()
        channels = [0] * 18                           # tableau RC complet

        self.set_ch(channels, self.ch_rl, self.u_to_pwm(u[0]))  # T1
        self.set_ch(channels, self.ch_rr, self.u_to_pwm(u[1]))  # T2
        self.set_ch(channels, self.ch_fl, self.u_to_pwm(u[2]))  # T3
        self.set_ch(channels, self.ch_fr, self.u_to_pwm(u[3]))  # T4

        rc.channels = channels
        self.pub_rc.publish(rc)


def main():
    rclpy.init()
    node = RCAllocationB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
