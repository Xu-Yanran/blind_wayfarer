"""
Blind-Wayfarer module for the robot : Forest-Pledge
"""

import os
import csv
from datetime import datetime
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

import time

# Hardware abstraction
from hal import Hal


class BlindWayfarer(Node):
    def __init__(self):
        super().__init__('blind_wayfarer')
        
        # Configuration
        self.speed = 0.3
        self.max_speed = 0.5
        self.min_speed = 0.0
        self.turning_angle = 30  # degrees
        self.turning_tolerance = 5  # degrees
        self.probing_time = 1.5  # second
        self.GB_time = 1.5       # second
        self.safety_pitch = 15   # degree

        # PID parameter
        self.Kp = 0.015

        # High-level state
        self.angle_counter = 0
        self.state = "0"  # "0" or "1"

        # Hardware layer
        self.hal = Hal(self)

        # Set the waypoint direction from hardware
        self.wp_direction = self.hal.get_yaw()

        # Publisher to command velocities
        self.vel_pub = self.create_publisher(Vector3, 'vel_cmd', 1)

    # -------------------------------------------------------------------------
    # Safety / Checking
    # -------------------------------------------------------------------------
    def check_pitch_safety(self):
        """
        Check that pitch angle is below threshold to avoid flipping.
        """
        pitch_angle = self.hal.get_pitch()
        return (pitch_angle <= self.safety_pitch)

    # -------------------------------------------------------------------------
    # Motion Commands
    # -------------------------------------------------------------------------
    def send_vel_cmd(self, vel_cmd):
        """
        Publish velocity command [left_speed, right_speed].
        Enforce min/max speed constraints.
        """
        msg = Vector3()
        msg.x = vel_cmd[0]
        msg.y = vel_cmd[1]

        # Enforce minimum speeds
        if 0 < msg.x < self.min_speed:
            msg.x = self.min_speed
        elif 0 > msg.x > -self.min_speed:
            msg.x = -self.min_speed

        if 0 < msg.y < self.min_speed:
            msg.y = self.min_speed
        elif 0 > msg.y > -self.min_speed:
            msg.y = -self.min_speed

        # Enforce maximum speeds
        if msg.x > self.max_speed:
            msg.x = self.max_speed
        elif msg.x < -self.max_speed:
            msg.x = -self.max_speed

        if msg.y > self.max_speed:
            msg.y = self.max_speed
        elif msg.y < -self.max_speed:
            msg.y = -self.max_speed

        self.vel_pub.publish(msg)

    def move_forward(self):
        start_time = time.time()
        while time.time() - start_time < self.probing_time:
            rclpy.spin_once(self, timeout_sec=0.2)
            self.send_vel_cmd([self.speed, self.speed])
            if self.hal.is_stuck() or not self.check_pitch_safety():
                break

    def step_back(self):
        start_time = time.time()
        while time.time() - start_time < self.GB_time:
            self.send_vel_cmd([-self.speed, -self.speed])

    def rotate_right(self):
        """
        Turn right by self.turning_angle degrees and update angle_counter.
        """
        initial_yaw = self.hal.get_yaw()
        target_direction = (initial_yaw + self.turning_angle) % 360
        self.rotate(0.0, target_direction)

        turned_angle = calculate_circular_difference(
            initial_yaw, self.hal.get_yaw()
        )
        self.angle_counter += turned_angle

    def rotate_back(self):
        """
        According to the Pledge algorithm, rotate left until angle_counter <= 0.
        """
        prev_yaw = self.hal.get_yaw()
        while self.angle_counter > 0:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Turn left gradually
            self.send_vel_cmd([self.speed * 0.75, self.speed / 0.75])

            turned_angle = calculate_circular_difference(
                prev_yaw, self.hal.get_yaw()
            )
            self.angle_counter += turned_angle
            prev_yaw = self.hal.get_yaw()

            if self.hal.is_stuck() or not self.check_pitch_safety():
                self.state = "1"
                return
        
        self.angle_counter = 0

    def rotate(self, base_speed, target_direction):
        """
        Rotate to target direction:
          1) Try turning one side forward
          2) If stuck, try the other side
        """
        # Phase 1: rotate by moving one side forward
        while True:
            rclpy.spin_once(self, timeout_sec=0.2)

            angle_diff = calculate_circular_difference(
                self.hal.get_yaw(), target_direction
            )
            k = self.Kp * angle_diff

            if base_speed == 0:
                if k > 0:
                    vel_left = k
                    vel_right = 0.0
                else:
                    vel_left = 0.0
                    vel_right = -k
            else:
                vel_left = base_speed + k
                vel_right = -base_speed - k

            self.send_vel_cmd([vel_left, vel_right])

            if abs(angle_diff) <= self.turning_tolerance:
                self.state = "0"
                return
            if self.hal.is_stuck():
                self.state = "1"
                break

        # Phase 2: rotate by moving the other side backward
        while True:
            rclpy.spin_once(self, timeout_sec=0.2)

            angle_diff = calculate_circular_difference(
                self.hal.get_yaw(), target_direction
            )
            k = self.Kp * angle_diff

            if base_speed == 0:
                if k > 0:
                    vel_left = 0.0
                    vel_right = -k
                else:
                    vel_left = k
                    vel_right = 0.0
            else:
                vel_left = base_speed + k
                vel_right = -base_speed - k

            self.send_vel_cmd([vel_left, vel_right])

            if abs(angle_diff) <= self.turning_tolerance:
                self.state = "0"
                return
            if self.hal.is_stuck():
                self.state = "1"
                break

    def get_max_speed(self):
        return self.max_speed

# -------------------------------------------------------------------------
# Main Loop
# -------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    blind_wayfarer = BlindWayfarer()

    max_speed = blind_wayfarer.get_max_speed()
    speed = blind_wayfarer.speed

    # Set up logging
    curr_time = datetime.now().strftime("%y_%m_%d_%H_%M")
    log_dir = f"/home/gdp/field_logs/{curr_time}"
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, "log_data.tsv")

    vel_cmd = [0.0, 0.0]

    with open(log_file, mode="a", newline="") as file:
        writer = csv.writer(file, delimiter='\t')
        try:
            while True:
                # Spin once to process new messages
                rclpy.spin_once(blind_wayfarer, timeout_sec=0.1)

                # Check for emergencies
                if blind_wayfarer.hal.is_stuck() or not blind_wayfarer.check_pitch_safety():
                    blind_wayfarer.state = "1"

                if blind_wayfarer.state == "0":
                    # Normal (Waypoint) mode
                    if blind_wayfarer.angle_counter <= 0:
                        curr_yaw = blind_wayfarer.hal.get_yaw()
                        angle_diff = calculate_circular_difference(
                            curr_yaw, blind_wayfarer.wp_direction
                        )
                        k = blind_wayfarer.Kp * angle_diff
                        vel_left = speed + k
                        vel_right = speed - k

                        # Clip speeds
                        vel_left = max(-max_speed, min(max_speed, vel_left))
                        vel_right = max(-max_speed, min(max_speed, vel_right))

                        vel_cmd = [vel_left, vel_right]
                        blind_wayfarer.send_vel_cmd(vel_cmd)
                    else:
                        # In the middle of a Pledge-like boundary follow
                        blind_wayfarer.rotate_back()

                elif blind_wayfarer.state == "1":
                    # Obstacle or tilt detected -> step back & rotate right
                    blind_wayfarer.step_back()
                    blind_wayfarer.rotate_right()

                # Log data
                writer.writerow([
                    datetime.now().strftime("%H%M_%S"),   # Timestamp
                    round(vel_cmd[0], 2),                # Left velocity
                    round(vel_cmd[1], 2),                # Right velocity
                    round(blind_wayfarer.hal.get_yaw(), 2),  # Current yaw
                    blind_wayfarer.hal.get_latest_flow_value(),  # Flow
                    blind_wayfarer.hal.get_flow_quality()       # Flow quality
                ])

        except:
            # On any error, perform emergency stop
            blind_wayfarer.hal.emergency_stop()


def calculate_circular_difference(angle1, angle2):
    """
    Calculates the smallest difference between two angles in the range [0, 360].
    The result is the shortest path around the circle, either clockwise or counterclockwise.

    Args:
        angle1 (float): The first angle in degrees (0-360).
        angle2 (float): The second angle in degrees (0-360).

    Returns:
        float: The smallest circular difference between the two angles, in the range [-180, 180].
    """
    diff = (angle2 - angle1) % 360
    if diff > 180:
        diff -= 360
    return diff


if __name__ == '__main__':
    main()