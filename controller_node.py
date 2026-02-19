#!/usr/bin/env python3
"""
Interface de contrôle du robot PlaSMAR.
Gère la machine à états (Manuel / Auto) et la priorité des commandes clavier.
"""


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
import sys
import termios
import tty
import select
import time
import math

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        # Paramètres de vitesse pour le mode manuel
        self.declare_parameter('forward_speed', 20.0)
        self.declare_parameter('yaw_speed', 5.0)
        self.declare_parameter('vertical_speed', 20.0)
        
        self.forward_speed = self.get_parameter('forward_speed').value
        self.yaw_speed = self.get_parameter('yaw_speed').value
        self.vertical_speed = self.get_parameter('vertical_speed').value
        
        # État
        self.mode = 'MANUAL'
        self.speed_multiplier = 1.0
        
        # Commandes stockées
        self.manual_twist = Twist() # Commande clavier
        self.auto_twist = Twist()   # Commande du tracking
        
        self.last_key_time = time.time()
        self.last_mode_switch = 0.0
        
        # Configuration clavier
        try:
            self.settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except Exception:
            self.get_logger().warn("Impossible de configurer le clavier (mode non-interactif ?)")
            self.settings = None
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/plasmar/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/plasmar/mode', 10)
        
        # Appelle le noeud traj_tracking_node
        self.auto_trigger_pub = self.create_publisher(Bool, '/plasmar/auto_trigger', 10)
        
        # Subscribers
        self.create_subscription(Twist, '/plasmar/auto_cmd_vel', self.auto_cmd_callback, 10)
        
        self.create_subscription(PoseStamped, '/plasmar/target', self.target_callback, 10)
        
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        print("\r\n" + "=" * 60)
        print("\rCONTRÔLEUR PLASMAR")
        print("\r[M] : Basculer Auto/Manuel")
        print("\r[ESPACE] ou [ZQSD] : Reprise MANUELLE immédiate")
        print("\r[Ctrl+C] : Quitter")
        print("\r" + "=" * 60 + "\r\n")
        self.print_help()

    def target_callback(self, msg):
        pass

    def auto_cmd_callback(self, msg):
        self.auto_twist = msg

    def print_help(self):
        help_text = """
\r┌─────────────────────────────────────────────────────────┐
\r│              COMMANDES MODE MANUEL                      │
\r├─────────────────────────────────────────────────────────┤
\r│  Z : Avancer          │  Q : Tourner Gauche             │
\r│  S : Reculer          │  D : Tourner Droite             │
\r│  A : Monter           │  E : Descendre                  │
\r│  J : Latéral Gauche   │  L : Latéral Droit              │
\r│  + : Accélérer        │  - : Ralentir                   │
\r├─────────────────────────────────────────────────────────┤
\r│              COMMANDES GLOBALES                         │
\r├─────────────────────────────────────────────────────────┤
\r│  ESPACE : STOP (Reprise main immédiate)                 │
\r│  M      : BASCULER MODE AUTO / MANUEL                   │
\r│  Ctrl+C : Quitter                                       │
\r└─────────────────────────────────────────────────────────┘
        """
        print(help_text)

    def print_status(self):
        msg = f'\r[STATUS] Mode: {self.mode} | Vitesse: {self.speed_multiplier:.1f}x '
        if self.mode == 'AUTO':
            if abs(self.auto_twist.linear.x) > 0.001:
                msg += "| AUTO ACTIF (Cmd Reçue)"
            else:
                msg += "| AUTO ATTENTE..."
        sys.stdout.write(msg)
        sys.stdout.flush()

    def get_key(self):
        if self.settings is None: return None
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            key = sys.stdin.read(1)
            self.last_key_time = time.time()
            return key
        return None

    def process_key(self, key):
        if key is None: return True
        k = key.lower()
        now = time.time()

        # Bascule mode auto/manuel
        if k == 'm':
            if (now - self.last_mode_switch) < 0.5: return True
            self.last_mode_switch = now
            if self.mode == 'MANUAL':
                self.set_auto_mode()
            else:
                self.set_manual_mode()
            return True

        # Arrêt d'urgence
        if k == ' ' or k == '\x1b':
            self.set_manual_mode()
            if k == '\x1b': return True
            return True
            
        if k == '\x03':
            return False

        # Reprise en main par l'opérateur si mouvement clavier détecté en mode Auto
        movement_keys = ['z', 's', 'q', 'd', 'a', 'e']
        if self.mode == 'AUTO' and k in movement_keys:
            print("\r\n[ALERTE] Touche pressée -> Reprise MANUELLE immédiate !")
            self.set_manual_mode()

        # Consignes manuelles
        if self.mode == 'MANUAL':
            new_twist = Twist()
            speed = self.forward_speed * self.speed_multiplier
            yaw_s = self.yaw_speed * self.speed_multiplier
            vert_s = self.vertical_speed * self.speed_multiplier
            
            if k == 'z': new_twist.linear.x = speed
            elif k == 's': new_twist.linear.x = -speed
            elif k == 'j': new_twist.linear.y = speed
            elif k == 'l': new_twist.linear.y = -speed
            elif k == 'q': new_twist.angular.z = yaw_s
            elif k == 'd': new_twist.angular.z = -yaw_s
            elif k == 'a': new_twist.linear.z = vert_s
            elif k == 'e': new_twist.linear.z = -vert_s
            elif k == '+': self.speed_multiplier = min(3.0, self.speed_multiplier + 0.1)
            elif k == '-': self.speed_multiplier = max(0.1, self.speed_multiplier - 0.1)
            
            self.manual_twist = new_twist
            
        return True

    def set_auto_mode(self):
        self.mode = 'AUTO'
        self.manual_twist = Twist()
        self.mode_pub.publish(String(data='AUTO'))
        self.auto_trigger_pub.publish(Bool(data=True))
        print("\r\n PILOTE AUTOMATIQUE ACTIVÉ \r")

    def set_manual_mode(self):
        self.mode = 'MANUAL'
        self.mode_pub.publish(String(data='MANUAL'))
        self.auto_trigger_pub.publish(Bool(data=False))
        self.manual_twist = Twist()
        self.cmd_pub.publish(self.manual_twist)
        print("\r\n MODE MANUEL ACTIVÉ \r")

    def control_loop(self):
        try:
            key = self.get_key()
            if not self.process_key(key):
                raise KeyboardInterrupt
            
            final_cmd = Twist()

            if self.mode == 'MANUAL':
                # Watchdog : coupe les moteurs si aucune touche pressée
                if key is None and (time.time() - self.last_key_time > 0.6):
                    self.manual_twist = Twist()
                final_cmd = self.manual_twist

            elif self.mode == 'AUTO':
                final_cmd = self.auto_twist

            self.cmd_pub.publish(final_cmd)

        except Exception:
            pass

    def restore_settings(self):
        if self.settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_settings()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        print("\r\nArrêt du contrôleur.\r\n")

if __name__ == '__main__':
    main()
