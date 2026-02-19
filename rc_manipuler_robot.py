#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mavros_msgs.msg import State, OverrideRCIn


def clamp(x, lo, hi):             # Fonction saturation
    return max(lo, min(hi, x)) 


class RCMixer4ThrustersServo7(Node):

    def __init__(self):
        super().__init__('rc_mixer_4thrusters_servo7') 

        self.declare_parameter('ch_rear_left', 1)     # Thruster arrière gauche
        self.declare_parameter('ch_rear_right', 2)    # Thruster arrière droit
        self.declare_parameter('ch_front_left', 3)    # Thruster avant gauche
        self.declare_parameter('ch_front_right', 4)   # Thruster avant droit
        self.declare_parameter('ch_servo', 7)         # Servo

    
        self.declare_parameter('thr_pwm_center', 1500)  # PWM neutre des thrusters
        self.declare_parameter('thr_pwm_scale', 400.0)  # Amplitude PWM max autour du neutre
        self.declare_parameter('thr_pwm_min', 1100)     # Limite basse PWM thruster
        self.declare_parameter('thr_pwm_max', 1900)     # Limite haute PWM thruster

        
        self.declare_parameter('servo_pwm_center', 1500)  # Position neutre du servo
        self.declare_parameter('servo_pwm_up', 1700)      # PWM élevé du servo
        self.declare_parameter('servo_pwm_down', 1300)    # PWM bas du servo

    
        self.declare_parameter('deadband', 0.05)     # Zone morte pour éviter le bruit
        self.declare_parameter('rate_hz', 20.0)      # Fréquence d’envoi RC

        # Lecture des paramètres
        self.ch_rl = int(self.get_parameter('ch_rear_left').value)     
        self.ch_rr = int(self.get_parameter('ch_rear_right').value)    
        self.ch_fl = int(self.get_parameter('ch_front_left').value)    
        self.ch_fr = int(self.get_parameter('ch_front_right').value)   
        self.ch_servo = int(self.get_parameter('ch_servo').value)      

        self.thr_center = int(self.get_parameter('thr_pwm_center').value)  
        self.thr_scale = float(self.get_parameter('thr_pwm_scale').value)  
        self.thr_min = int(self.get_parameter('thr_pwm_min').value)        
        self.thr_max = int(self.get_parameter('thr_pwm_max').value)        

        self.servo_center = int(self.get_parameter('servo_pwm_center').value)  
        self.servo_up = int(self.get_parameter('servo_pwm_up').value)          
        self.servo_down = int(self.get_parameter('servo_pwm_down').value)      
 
        self.deadband = float(self.get_parameter('deadband').value)   
        rate_hz = float(self.get_parameter('rate_hz').value)         

        
        self.connected = False          # Indique si MAVROS est connecté
        self.last_cmd = Twist()         # Dernière commande clavier reçue

        
        self.pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)  # Publication RC override
        self.create_subscription(State, '/mavros/state', self.on_state, 10)       # Abonnement état MAVROS
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)          # Abonnement commandes clavier

        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)  # Boucle périodique d’envoi PWM

        self.get_logger().info(
            "RC Mixer ready:\n"
            f"  Rear:  ch{self.ch_rl}, ch{self.ch_rr}\n"
            f"  Front: ch{self.ch_fl}, ch{self.ch_fr}\n"
            f"  Servo: ch{self.ch_servo}\n"
            f"  Thr PWM: center={self.thr_center} scale={self.thr_scale} [{self.thr_min}..{self.thr_max}]\n"
            f"  Servo PWM: center={self.servo_center} up={self.servo_up} down={self.servo_down}\n"
            "  Inputs: /cmd_vel (linear.x, angular.z, linear.z)"
        )  # Message de debug au lancement

    def on_state(self, msg: State):
        self.connected = bool(msg.connected)  # Vrai uniquement si MAVROS est bien connecté au FCU

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg  # Sauvegarde de la dernière commande Twist

    def u_to_pwm_thruster(self, u: float) -> int:
        u = clamp(u, -1.0, 1.0)  # Saturation de u dans [-1, 1]
        pwm = int(self.thr_center + self.thr_scale * u)  # Conversion commande en PWM
        return int(clamp(pwm, self.thr_min, self.thr_max))  # Saturation du PWM dans les limites

    def on_timer(self):
        if not self.connected:
            return  # Ne rien envoyer si MAVROS n’est pas connecté

        surge = clamp(self.last_cmd.linear.x, -1.0, 1.0)  # Avance / recul
        yaw = clamp(self.last_cmd.angular.z, -1.0, 1.0)     # Rotation sur place

        heave_raw = self.last_cmd.linear.z                                # Montée / descente brute
        heave_mag = clamp(abs(heave_raw), 0.0, 1.0)       # Intensité sans signe

        if abs(surge) < self.deadband:
            surge = 0.0  # Supprime le bruit en avance
        if abs(yaw) < self.deadband:
            yaw = 0.0    # Supprime le bruit en rotation
        if abs(heave_raw) < self.deadband:
            heave_mag = 0.0  # Supprime le bruit vertical

        # Thrusters arrière : avance + rotation différentielle
        u_rl = surge + yaw   # Arrière gauche
        u_rr = surge - yaw   # Arrière droit

        # Thrusters avant : avance + poussée verticale
        u_fl = surge + heave_mag
        u_fr = surge + heave_mag

        u_rl = clamp(u_rl, -1.0, 1.0)  # Saturation des commandes dans [-1, 1]
        u_rr = clamp(u_rr, -1.0, 1.0)
        u_fl = clamp(u_fl, -1.0, 1.0)
        u_fr = clamp(u_fr, -1.0, 1.0)

        if heave_mag == 0.0:
            servo_pwm = self.servo_center        # Servo neutre
        else:
            servo_pwm = self.servo_up if heave_raw > 0.0 else self.servo_down  # Oriente le servo

        rc = OverrideRCIn()          # Message MAVROS
        channels = [0] * 18          # Tableau RC complet

        def set_ch(ch_1based: int, pwm: int):
            idx = ch_1based - 1      
            if 0 <= idx < 18:
                channels[idx] = int(pwm)  # Écriture sécurisée du PWM

        set_ch(self.ch_rl, self.u_to_pwm_thruster(u_rl))  # Thruster arrière gauche
        set_ch(self.ch_rr, self.u_to_pwm_thruster(u_rr))  # Thruster arrière droit
        set_ch(self.ch_fl, self.u_to_pwm_thruster(u_fl))  # Thruster avant gauche
        set_ch(self.ch_fr, self.u_to_pwm_thruster(u_fr))  # Thruster avant droit
        set_ch(self.ch_servo, servo_pwm)                  # Servo

        rc.channels = channels
        self.pub.publish(rc) 

def main():
    rclpy.init()
    node = RCMixer4ThrustersServo7()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

