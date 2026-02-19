#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool, String
# from mavros_msgs.msg import OverrideRCIn  # Commenté pour simulation
from geometry_msgs.msg import Twist

class PlasmarDriver(Node):
    def __init__(self):
        super().__init__("plasmar_driver")
        
        # === États ===
        self.mode = 'MANUAL'
        self.emergency_mode = False # Variable de sécurité
        self.current_twist = Twist()

        # === Publishers ===
        # Commenté car nous n'avons pas besoin de parler à MAVROS en simulation
        # self.rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        
        # === Subscribers ===
        self.create_subscription(Twist, '/plasmar/cmd_vel', self.vel_callback, 10)
        self.create_subscription(String, '/plasmar/mode', self.mode_callback, 10)
        
        # Nouvel abonnement : Écoute l'ordre d'urgence du Mission Node
        self.create_subscription(Bool, '/plasmar/emergency_surface', self.emergency_callback, 10)

        self.create_timer(0.05, self.control_loop) # 20Hz
        self.get_logger().info(" Driver PlaSMAR prêt (Mode Simulation sans MAVROS)")

    def emergency_callback(self, msg):
        """Déclenché par le Mission Node en cas de fuite ou batterie critique"""
        if msg.data:
            self.emergency_mode = True
            self.get_logger().error(" MODE URGENCE ACTIVÉ : Remontée forcée !")
        else:
            self.emergency_mode = False

    def mode_callback(self, msg):
        self.mode = msg.data

    def vel_callback(self, msg):
        self.current_twist = msg

    def control_loop(self):
        # On garde la logique de calcul pour voir les logs, même sans envoyer le message MAVROS
        # rc = OverrideRCIn()
        channels = [1500] * 18

        if self.emergency_mode:
            # === PROTOCOLE DE SURVIE ===
            channels[4] = 1500 # Forward stop
            channels[5] = 1500 # Forward stop
            channels[7] = 1500 # Servo anneau au neutre
            
            # 2. Forcer la remontée verticale (Canal 3)
            channels[2] = 1900
            
            self.get_logger().warn("Remontée d'urgence simulée (Vertical PWM: 1900)")
        else:
            # === PILOTAGE NORMAL (Manuel ou Auto) ===
            # Servo anneau (Canal 8)
            servo_val = 1500 + (self.current_twist.angular.z * 400)
            channels[7] = int(np.clip(servo_val, 1100, 1900))

            # Propulsion (Canaux 5 et 6)
            fwd_pwm = 1500 + (self.current_twist.linear.x * 400)
            channels[4] = int(np.clip(fwd_pwm, 1100, 1900))
            channels[5] = int(np.clip(fwd_pwm, 1100, 1900))

            # Verticalité (Canal 3)
            vert_pwm = 1500 + (self.current_twist.linear.z * 400)
            channels[2] = int(np.clip(vert_pwm, 1100, 1900))

        # self.rc_pub.publish(rc) # Commenté pour simulation

def main(args=None):
    rclpy.init(args=args)
    node = PlasmarDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
