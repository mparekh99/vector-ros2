#!/usr/bin/env python3
import anki_vector
from anki_vector.util import degrees
import sys
import termios
import tty
import select
import time
import math

# =============================
# Arrow key mapping (escape sequences)
# =============================
ARROW_KEYS = {
    '\x1b[A': "forward",   # UP
    '\x1b[B': "backward",  # DOWN
    '\x1b[C': "right",     # RIGHT
    '\x1b[D': "left"       # LEFT
}

# =============================
# Motor constants
# =============================
MAX_FWD_SPEED = 100  # mm/s
MAX_TURN_SPEED = 100 # mm/s
CONTROL_DT = 0.05    # loop delay in seconds

def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

# =============================
# Teleop main loop
# =============================
def teleop_loop(robot):
    old_attrs = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print("[INFO] Vector ready for teleop (arrow keys). Press ESC to quit.")

    try:
        while True:
            # Check if a key has been pressed
            if select.select([sys.stdin], [], [], 0.05)[0]:
                c = sys.stdin.read(3)
                if c == '\x1b':  # ESC key to quit
                    print("[INFO] Exiting teleop...")
                    break

                command = ARROW_KEYS.get(c, None)

                # Send motor commands based on arrow key
                if command == "forward":
                    robot.motors.set_wheel_motors(MAX_FWD_SPEED, MAX_FWD_SPEED)
                elif command == "backward":
                    robot.motors.set_wheel_motors(-MAX_FWD_SPEED, -MAX_FWD_SPEED)
                elif command == "left":
                    robot.motors.set_wheel_motors(-MAX_TURN_SPEED, MAX_TURN_SPEED)
                elif command == "right":
                    robot.motors.set_wheel_motors(MAX_TURN_SPEED, -MAX_TURN_SPEED)
                else:
                    robot.motors.set_wheel_motors(0, 0)
            else:
                # No key pressed, stop motors
                robot.motors.set_wheel_motors(0, 0)

            time.sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt detected, stopping robot.")

    finally:
        robot.motors.set_wheel_motors(0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)


# =============================
# Main
# =============================
def main():
    # Connect to Vector (replace with your serial if needed)
    with anki_vector.Robot("00706c20") as robot:
        # Initialize head and lift
        robot.behavior.set_head_angle(degrees(7.0))
        robot.behavior.set_lift_height(0.0)

        # Start teleop loop
        teleop_loop(robot)

if __name__ == "__main__":
    main()
