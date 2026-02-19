#!/usr/bin/env python3
import rclpy
from rclpy.node import Node                    

from geometry_msgs.msg import Twist            
from mavros_msgs.msg import State, OverrideRCIn 


def clamp(x, lo, hi):                           # Fonction saturation
    return max(lo, min(hi, x))                  


class RCThrusterNode(Node): #Nœud ROS2 pour piloter un propulseur via MAVROS RC override.
    
    def __init__(self):
        super().__init__('rc_thruster_node')    

        
        self.declare_parameter('rc_channel', 1)    # Canal RC du thruster (1 = AUX1)
        self.declare_parameter('pwm_center', 1500) # PWM neutre du thruster (1500µs)
        self.declare_parameter('pwm_min', 1100)    # Limite basse PWM
        self.declare_parameter('pwm_max', 1900)    # Limite haute PWM
        self.declare_parameter('rate_hz', 20.0)    # Fréquence d’envoi des commandes

        self.rc_channel = int(self.get_parameter('rc_channel').value)   
        self.pwm_center = int(self.get_parameter('pwm_center').value)   
        self.pwm_min = int(self.get_parameter('pwm_min').value)
        self.pwm_max = int(self.get_parameter('pwm_max').value) 
        rate_hz = float(self.get_parameter('rate_hz').value) 

        
        self.connected = False                # Vrai si MAVROS est connecté
        self.last_cmd = Twist()               # Dernière commande reçue sur /cmd_vel

        self.pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)  # Publication RC override
        self.create_subscription(State, '/mavros/state', self.on_state, 10)       # Abonnement état MAVROS
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)          # Abonnement commandes clavier

        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)  # Boucle périodique d’envoi PWM

        self.get_logger().info(
            f"RCThrusterNode OK. RC{self.rc_channel} from /cmd_vel.linear.x | "
            f"PWM [{self.pwm_min}..{self.pwm_max}] center={self.pwm_center}"
        )

    def on_state(self, msg: State):
        self.connected = bool(msg.connected)  # Vrai uniquement si MAVROS est bien connecté au FCU

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg  # Stocke la dernière commande clavier reçue

    def on_timer(self):
        if not self.connected:  # ne rien envoyer si MAVROS n’est pas connecté
            return

        u = self.last_cmd.angular.z  # On utilise la rotation Z pour piloter le servo
        u = clamp(u, -1.0, 1.0)      # Saturation de la commande U dans [-1, 1]

        pwm = int(self.pwm_center + 400.0 * u)  # Conversion commande → PWM
        pwm = clamp(pwm, self.pwm_min, self.pwm_max)  # Saturation du PWM dans les limites

        rc = OverrideRCIn()  # Message MAVROS RC override

        channels = [0] * 18  # Tableau des 18 canaux RC
        idx = self.rc_channel - 1  # Conversion canal RC (index 1 RC = index 0 en Python) 
        if 0 <= idx < 18:
            channels[idx] = pwm  # Écriture du PWM uniquement sur le canal du thruster
        rc.channels = channels 

        self.pub.publish(rc)


def main():
    rclpy.init()
    node = RCThrusterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()