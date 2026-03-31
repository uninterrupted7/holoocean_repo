#!/usr/bin/env python3
"""
vessel_teleop_node.py
=====================
Keyboard teleoperation node for the HoloOcean SurfaceVessel.

Publishes geometry_msgs/Twist to /holoocean/cmd_vel

Controls:
  w / s      → forward / backward (toggle on/off)
  a / d      → turn left / right (toggle on/off)
  SPACE      → emergency stop + reset heading
  q          → quit
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
HoloOcean Surface Vessel Teleop
─────────────────────────────────────
  w        forward (toggle)
  s        backward (toggle)
  a        turn left (toggle)
  d        turn right (toggle)
  SPACE    stop + reset
  q        quit
─────────────────────────────────────
"""

DEFAULT_LINEAR_SCALE = 1.0
DEFAULT_ANGULAR_SCALE = 0.5


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class VesselTeleopNode(Node):
    def __init__(self):
        super().__init__('vessel_teleop')
        self.declare_parameter('linear_scale', DEFAULT_LINEAR_SCALE)
        self.declare_parameter('angular_scale', DEFAULT_ANGULAR_SCALE)
        self._linear  = self.get_parameter('linear_scale').value
        self._angular = self.get_parameter('angular_scale').value
        
        self._forward = 0.0
        self._turn = 0.0
        
        self._pub = self.create_publisher(Twist, '/holoocean/cmd_vel', 10)
        self._timer = self.create_timer(0.1, self._publish_cmd)
        print(HELP)

    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = self._forward
        msg.angular.z = self._turn
        self._pub.publish(msg)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = get_key(settings)
                
                if key == 'q':
                    break
                elif key == 'w':
                    self._forward = self._linear if self._forward != self._linear else 0.0
                    print(f'Forward: {self._forward}')
                elif key == 's':
                    self._forward = -self._linear if self._forward != -self._linear else 0.0
                    print(f'Backward: {self._forward}')
                elif key == 'a':
                    self._turn = self._angular if self._turn != self._angular else 0.0
                    print(f'Turn: {self._turn}')
                elif key == 'd':
                    self._turn = -self._angular if self._turn != -self._angular else 0.0
                    print(f'Turn: {self._turn}')
                elif key == ' ':
                    self._forward = 0.0
                    self._turn = 0.0
                    print('STOP!')
        finally:
            self._pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = VesselTeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
