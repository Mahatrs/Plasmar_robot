#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time

class PlasmarTestBench(Node):
    def __init__(self):
        super().__init__('test_unit_node')
        self.pub = self.create_publisher(Twist, '/plasmar/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        print("\r\n" + "=" * 60)
        print("\r  BANC DE TEST PLASMAR ")
        print("\r" + "=" * 60)
        print("\r[1] TEST STABILITÉ (0 Effort pendant 10s)")
        print("\r    -> A faire avec init Pitch/Roll = 90 deg dans la physique")
        print("\r[2] TEST UNITAIRE : AVANCE SEULE (Surge 20N)")
        print("\r[3] TEST UNITAIRE : LATÉRAL SEUL (Sway 20N)")
        print("\r[4] SÉQUENCE COMPLÈTE ")
        print("\r    Avance 5s -> Stop -> Latéral 5s -> Stop -> Plonge 5s -> Avance 4s -> Remonte 5s")
        print("\r[q] Quitter (Stop robot)")
        print("\r" + "=" * 60)
        
        self.timer = self.create_timer(0.1, self.key_loop)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def key_loop(self):
        key = self.get_key()
        if key == '1': self.run_passive_stability()
        elif key == '2': self.run_surge_only()
        elif key == '3': self.run_sway_only()
        elif key == '4': self.run_full_sequence()
        elif key in ['q', '\x03']:
            self.send_cmd(duration=0.5)
            rclpy.shutdown(); exit()

    def send_cmd(self, x=0.0, y=0.0, z=0.0, yaw=0.0, duration=1.0):
        """Envoie une commande constante pendant une durée donnée"""
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = float(x), float(y), float(z)
        msg.angular.z = float(yaw)
        
        rate = 20 # Hz
        steps = int(duration * rate)
        for _ in range(steps):
            self.pub.publish(msg)
            time.sleep(1.0/rate)
            
    def run_passive_stability(self):
        print("\r\n>>> TEST 1 : STABILITÉ (10s)...")
        self.send_cmd(duration=10.0)
        print("\r>>> FIN.")

    def run_surge_only(self):
        print("\r\n>>> TEST 2 : AVANCE PURE (5s)...")
        self.send_cmd(x=20.0, duration=5.0)
        self.send_cmd(duration=2.0)
        print("\r>>> FIN.")

    def run_sway_only(self):
        print("\r\n>>> TEST 3 : LATÉRAL PUR (5s)...")
        self.send_cmd(y=20.0, duration=5.0)
        self.send_cmd(duration=2.0)
        print("\r>>> FIN.")

    def run_full_sequence(self):
        print("\r\n>>> TEST 4 : SÉQUENCE COMPLÈTE...")
        
        print("\r 1. Avance (5s)")
        self.send_cmd(x=20.0, duration=5.0)
        
        print("\r 2. Arrêt (1s)")
        self.send_cmd(duration=1.0)
        
        print("\r 3. Décalage Latéral (5s) - Sans avancer")
        self.send_cmd(y=20.0, duration=5.0)
        
        print("\r 4. Arrêt (1s)")
        self.send_cmd(duration=1.0)
        
        print("\r 5. Plonge (5s)")
        self.send_cmd(z=-20.0, duration=5.0)
        
        print("\r 6. Arrêt (1s)")
        self.send_cmd(duration=1.0)
        
        print("\r 7. Avance Sous-marine (4s)")
        self.send_cmd(x=20.0, duration=4.0)
        
        print("\r 8. Remonte (5s)")
        self.send_cmd(z=20.0, duration=5.0)
        
        print("\r 9. Fin (Arrêt final)")
        self.send_cmd(duration=2.0)
        print("\r>>> SÉQUENCE TERMINÉE.")

def main(args=None):
    rclpy.init(args=args)
    node = PlasmarTestBench()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
