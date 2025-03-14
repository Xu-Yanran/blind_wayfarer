"""
Hardware Abstraction Layer (HAL) for the robot.
"""

import time
import sys

# Example hardware modules
import CMPS14
from controller import direct_drive

from geometry_msgs.msg import Vector3


class Hal:
    """
    Hardware Abstraction Layer for the rover:
      - Compass/IMU via CMPS14
      - Motor driver
      - Optical flow sensor
    """
    def __init__(self, node):
        """
        Args:
            node (rclpy.node.Node): The ROS2 node that will create subscriptions.
        """
        # Store a reference to the ROS node 
        self.node = node

        # 1) Initialize compass
        self.cp14 = CMPS14.CMPS14()
        self.cp14.Init()

        # 2) Initialize motor driver
        self.motor = direct_drive.Rover()

        # 3) Optical flow
        self.OF_window = 10
        self.of_list = [1] * self.OF_window
        self.of_quality = 0

        # Create a subscription for optical flow
        # (If your actual topic name/type differs, adjust below)
        self.optical_flow_sub = self.node.create_subscription(
            Vector3,
            'optical_flow',
            self.opticalflow_cb,
            1
        )

    # -------------------------------------------------------------------------
    # Compass / IMU Methods
    # -------------------------------------------------------------------------
    def get_yaw(self):
        """Return the 16-bit compass bearing from the CMPS14 sensor."""
        return self.cp14.GetCompassBearing16()

    def get_pitch(self):
        """Return pitch angle from the CMPS14 sensor, adjusting for negative wrap."""
        i2c_data = self.cp14.GetAllBasicData()
        pitch_angle = i2c_data[4]
        if pitch_angle > 127:
            pitch_angle -= 256
        return pitch_angle

    # -------------------------------------------------------------------------
    # Motor Methods
    # -------------------------------------------------------------------------
    def emergency_stop(self):
        """
        Immediately stop the rover's motors.
        """
        print("Emergency Stop")
        vel_cmd = [0.0, 0.0]
        self.motor.motor_control(vel_cmd)

    # -------------------------------------------------------------------------
    # Optical Flow Methods
    # -------------------------------------------------------------------------
    def opticalflow_cb(self, msg):
        """
        ROS callback for optical flow data.

        Updates flow buffer to determine if the rover is stuck.
        """
        flow_x = msg.x
        flow_y = msg.y
        self.of_quality = msg.z  # 0..255

        # Use whichever metric you like for "movement" 
        flow = abs(flow_x) + abs(flow_y)

        self.of_list.pop(0)
        self.of_list.append(flow)

    def is_stuck(self):
        """
        Returns True if the rover hasn't moved for the
        entire window of optical-flow readings.
        """
        return sum(self.of_list) == 0

    def get_latest_flow_value(self):
        """
        Returns the most recent flow reading for logging or debugging.
        """
        return self.of_list[-1]

    def get_flow_quality(self):
        """
        Returns the latest flow quality reading (0..255).
        """
        return self.of_quality