#!/usr/bin/env python3

import anki_vector
import time
import math

# ============================
# Parameters
# ============================

SIDE_LENGTH_MM = 150.0        # 15 cm square
ANGLE_TOL = math.radians(2)   # 2 degree tolerance
DIST_TOL = 2.0                # 2 mm tolerance

MAX_FWD_SPEED = 80            # mm/s
MAX_TURN_SPEED = 80           # mm/s

KP_TURN = 120                 # proportional gain for turning
KP_FWD = 1.5                  # proportional gain for forward

CONTROL_DT = 0.01             # 10 ms loop


# ============================
# Helpers
# ============================

def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def turn_90_deg(robot, direction=1):
    """
    direction = +1 for CCW
    direction = -1 for CW
    """

    start_angle = robot.pose.rotation.angle_z.radians
    target_angle = start_angle + direction * (math.pi / 2)

    print("Turning 90 degrees...")

    while True:
        current_angle = robot.pose.rotation.angle_z.radians
        error = wrap_angle(target_angle - current_angle)

        if abs(error) < ANGLE_TOL:
            break

        # Proportional control
        turn_speed = KP_TURN * error

        # Clamp speed
        turn_speed = max(-MAX_TURN_SPEED,
                         min(MAX_TURN_SPEED, turn_speed))

        robot.motors.set_wheel_motors(-turn_speed,
                                       turn_speed)

        time.sleep(CONTROL_DT)

    robot.motors.set_wheel_motors(0, 0)
    time.sleep(0.2)
    print("Turn complete.")


def drive_forward(robot, distance_mm):

    start_pose = robot.pose.position
    start_x = start_pose.x
    start_y = start_pose.y

    print("Driving forward...")

    while True:
        pose = robot.pose.position
        dx = pose.x - start_x
        dy = pose.y - start_y
        traveled = math.sqrt(dx*dx + dy*dy)

        error = distance_mm - traveled

        if abs(error) < DIST_TOL:
            break

        # Proportional control
        fwd_speed = KP_FWD * error

        # Clamp speed
        fwd_speed = max(-MAX_FWD_SPEED,
                         min(MAX_FWD_SPEED, fwd_speed))

        robot.motors.set_wheel_motors(fwd_speed,
                                      fwd_speed)

        time.sleep(CONTROL_DT)

    robot.motors.set_wheel_motors(0, 0)
    time.sleep(0.2)
    print("Forward complete.")


# ============================
# Main
# ============================

def main():

    with anki_vector.Robot() as robot:

        print("Starting closed-loop square test...")

        for i in range(4):
            print(f"\n--- Side {i+1} ---")
            drive_forward(robot, SIDE_LENGTH_MM)
            turn_90_deg(robot, direction=1)

        robot.motors.set_wheel_motors(0, 0)
        print("\nSquare complete!")


if __name__ == "__main__":
    main()
