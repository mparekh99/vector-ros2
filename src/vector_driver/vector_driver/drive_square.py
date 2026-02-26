#!/usr/bin/env python3

import anki_vector
import time
import math
from threading import Timer

# ============================
# Parameters
# ============================

MAX_FWD_SPEED = 80       # mm/s
MAX_TURN_SPEED = 80      # mm/s

CONTROL_DT = 0.01        # 10 ms loop
ANGLE_TOL = math.radians(2)
DIST_TOL = 2.0           # mm

# ============================
# Helpers
# ============================

def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

# ============================
# Motion Functions
# ============================

def turn_90_deg(robot, direction=1):
    start_angle = robot.pose.rotation.angle_z.radians
    target_angle = start_angle + direction * (math.pi / 2)

    while True:
        current_angle = robot.pose.rotation.angle_z.radians
        error = wrap_angle(target_angle - current_angle)

        if abs(error) < ANGLE_TOL:
            break

        turn_speed = max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, 120*error))
        robot.motors.set_wheel_motors(-turn_speed, turn_speed)
        time.sleep(CONTROL_DT)

    robot.motors.set_wheel_motors(0, 0)

def drive_forward(robot, distance_mm):
    start_pose = robot.pose.position
    start_x = start_pose.x
    start_y = start_pose.y

    while True:
        pose = robot.pose.position
        dx = pose.x - start_x
        dy = pose.y - start_y
        traveled = math.sqrt(dx*dx + dy*dy)
        error = distance_mm - traveled

        if abs(error) < DIST_TOL:
            break

        fwd_speed = max(-MAX_FWD_SPEED, min(MAX_FWD_SPEED, 1.5*error))
        robot.motors.set_wheel_motors(fwd_speed, fwd_speed)
        time.sleep(CONTROL_DT)

    robot.motors.set_wheel_motors(0, 0)


def do_nothing(robot):
    # Stay stiill

    time.sleep(3)

# ============================
# Timed wrapper
# ============================

def run_with_timer(robot, func, duration_sec=10, *args, **kwargs):
    """
    Run a function in a loop but stop after duration_sec.
    """
    stop_flag = {"stop": False}

    def timer_callback():
        stop_flag["stop"] = True

    # start the timer
    t = Timer(duration_sec, timer_callback)
    t.start()

    print(f"Starting timed run for {duration_sec} seconds...")

    while not stop_flag["stop"]:
        func(robot, *args, **kwargs)
        time.sleep(CONTROL_DT)

    robot.motors.set_wheel_motors(0, 0)
    print("Timed run complete.")

# ============================
# Main
# ============================

def main():
    with anki_vector.Robot("00706c20") as robot:
        # print("OOOOOOSHUIIIJIJIJI")

        # === Do your motion once ===
        # Example: run 100 mm forward
        # drive_forward(robot, distance_mm=100)

        # Example: do a 90 degree turn
        # turn_90_deg(robot, direction=1)

        # ==========================
        # After motion is done, just wait indefinitely
        # ==========================
        print("Motion complete, now idling until launch shutdown...")
        # try:
        #     while True:
        # turn_90_deg(robot)
        do_nothing(robot)
        drive_forward(robot, 100)
                # do_nothing(robot)  # just sleep in a tight loop
        # except KeyboardInterrupt:
        print("Exiting idle loop...")
        while True:

            robot.motors.set_wheel_motors(0, 0)



if __name__ == "__main__":
    main()
