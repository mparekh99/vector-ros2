#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty

class TeleopVector(Node):
    def __init__(self):
        super().__init__('teleop_vector')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch1 = sys.stdin.read(1)
            if ch1 == '\x1b':          # Escape sequence start
                ch2 = sys.stdin.read(1)
                ch3 = sys.stdin.read(1)
                return ch1 + ch2 + ch3
            else:
                return ch1
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self):
        twist = Twist()
        print("Use arrow keys to drive. Ctrl-C to quit.")
        while rclpy.ok():
            key = self.get_key()
            if key == '\x1b[A':    # Up
                twist.linear.x = 0.05
                twist.angular.z = 0.0
            elif key == '\x1b[B':  # Down
                twist.linear.x = -0.05
                twist.angular.z = 0.0
            elif key == '\x1b[C':  # Right
                twist.linear.x = 0.0
                twist.angular.z = -0.5
            elif key == '\x1b[D':  # Left
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopVector()
    try:
        teleop.run()
    except KeyboardInterrupt:
        print("\nExiting teleop")
    finally:
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
