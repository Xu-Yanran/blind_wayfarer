import pygame
import numpy as np
from abstract_forest_simulation import RobotSimulation
from scipy.stats import levy  # for generating Levy-distributed step lengths

class LevyWalkAgent:
    def __init__(self, sim):
        # Initialize the simulation instance and control parameters.
        self.sim = sim
        self.state = 0  # 0: normal driving, 1: collision recovery
        self.tolerance = 5
        
        # Variables specific to Levy Walk
        self.step_length = 12  # initial step length
        self.turn_done = False
        self.move_done = False
        self.desired_angle = self.sim.robot_angle  # initial desired angle
        
        # Reset simulation-specific control variables
        self.sim.waypoint_flag = True
        self.sim.distance_traveled = 0
        self.sim.angle_counter = 0

    def levy_flight_step_length(self):
        """Generate a step length using a heavy-tailed Levy distribution."""
        scale_factor = 1.0
        step = levy.rvs() * scale_factor
        # Clamp the step length between 12 and 200.
        if step < 12:
            step = 12
        elif step > 200:
            step = 200
        return step

    def LevyWalk(self):
        """Control algorithm based on Levy Walk."""
        collision_detected = self.sim.collision_check()

        if self.state == 0:
            # Normal driving: adjust heading to align with 90° within tolerance.
            if self.sim.robot_angle > 90 + self.tolerance and self.sim.robot_angle < 270:
                self.sim.rotate_right(wp_flag=True)
            elif self.sim.robot_angle < 90 - self.tolerance or self.sim.robot_angle >= 270:
                self.sim.rotate_left(wp_flag=True)
            else:
                self.sim.move_forward()

            # On collision, transition to collision recovery.
            if collision_detected[0] != "None":
                self.state = 1
                self.sim.waypoint_flag = False
                self.sim.distance_traveled = 0

                self.step_length = self.levy_flight_step_length()
                angle_change = np.random.uniform(-180, 180)  # random turn angle
                self.desired_angle = (self.sim.robot_angle + angle_change) % 360
                self.turn_done = False
                self.move_done = False

        elif self.state == 1:
            # If collision persists, back up and reset turning parameters.
            if collision_detected[0] != "None":
                self.sim.move_backward()
                self.step_length = self.levy_flight_step_length()
                angle_change = np.random.uniform(-180, 180)
                self.desired_angle = (self.sim.robot_angle + angle_change) % 360
                self.turn_done = False
                self.move_done = False

            # Execute turning toward the desired angle.
            if not self.turn_done:
                diff = self.desired_angle - self.sim.robot_angle
                if diff < 0:
                    diff += 360

                if diff < 180:
                    self.sim.rotate_left(wp_flag=True)
                else:
                    self.sim.rotate_right(wp_flag=True)

                if abs(diff) < 10:
                    self.turn_done = True

            # After turning, move forward until the step length is reached.
            if self.turn_done and not self.move_done:
                self.sim.move_forward()
                if self.sim.distance_traveled >= self.step_length:
                    self.move_done = True
                    self.state = 0

    def run(self):
        running = True
        while running:
            # Process Pygame events (e.g., window closing)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Run the Levy Walk control method.
            self.LevyWalk()

            # Update the simulation display.
            if self.sim.render:
                self.sim._render()
            self.sim.clock.tick(90)

            # Terminate if the simulation meets an end condition.
            if self.sim.robot_y < 0 or self.sim.lost_flag or self.sim.stuck_flag:
                running = False

        print("Simulation ended.")

if __name__ == "__main__":
    # Create a simulation instance. Adjust parameters as needed.
    sim = RobotSimulation(render=True, manual=False)
    agent = LevyWalkAgent(sim)
    agent.run()
