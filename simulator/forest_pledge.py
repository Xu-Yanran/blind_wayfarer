import pygame
import numpy as np
from abstract_forest_simulation import RobotSimulation

class ForestPledgeAgent:
    def __init__(self, sim):
        # Create the simulation with manual control off so our agent can drive the robot.
        self.sim = sim
        self.state = 0  # 0: normal driving, 1: collision recovery
        self.tolerance = 5
        self.step_size = 12

        # Make sure simulation-specific control variables are reset
        self.sim.waypoint_flag = True
        self.sim.distance_traveled = 0
        self.sim.angle_counter = 0

    def ForestPledge(self):
        # Get the current collision information from the simulation.
        collision_detected = self.sim.collision_check()

        # If the robot is roughly facing 90° (the destination direction),
        # reset the turning counter.
        if self.sim.robot_angle <= 90 + self.tolerance and self.sim.robot_angle >= 90 - self.tolerance:
            self.sim.angle_counter = 0

        # State 0: Normal driving with waypoint control.
        if self.state == 0:
            if not self.sim.waypoint_flag:
                # Global strategy: move forward until a predefined step is completed.
                self.sim.move_forward()
                if self.sim.distance_traveled >= self.step_size:
                    # If the turning counter is within tolerance, switch back to waypoint mode.
                    if self.sim.angle_counter < self.tolerance:
                        self.sim.waypoint_flag = True
                    else:
                        # Otherwise, perform a small recovery turn and continue moving.
                        self.sim.recovery_left()
                        self.sim.move_forward()
            else:
                # Waypoint mode: adjust heading to align with 90°.
                if self.sim.robot_angle > 90 + self.tolerance and self.sim.robot_angle < 270:
                    self.sim.rotate_right(wp_flag=True)
                elif self.sim.robot_angle < 90 - self.tolerance or self.sim.robot_angle >= 270:
                    self.sim.rotate_left(wp_flag=True)
                else:
                    self.sim.move_forward()

            # If any collision is detected, switch to state 1 (collision recovery).
            if collision_detected[0] != "None":
                self.state = 1
                self.sim.waypoint_flag = False
                self.sim.distance_traveled = 0

        # State 1: Collision recovery.
        elif self.state == 1:
            self.sim.move_backward()
            self.sim.rotate_right()
            if collision_detected[0] == "None":
                self.state = 0

    def run(self):
        running = True
        while running:
            # Process Pygame events (e.g. to allow window closing)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Run the Forest Pledge control method.
            self.ForestPledge()

            # Update the simulation display.
            if self.sim.render:
                self.sim._render()
            self.sim.clock.tick(90)

            # Terminate if the simulation has reached an end condition.
            if self.sim.robot_y < 0 or self.sim.lost_flag or self.sim.stuck_flag:
                running = False

        print("Simulation ended.")

if __name__ == "__main__":
    agent = ForestPledgeAgent()
    agent.run()
