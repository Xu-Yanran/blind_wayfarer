import os
import sys
import pygame
import random
import math
import time
import numpy as np
from matplotlib import pyplot as plt
import argparse

from abstract_forest_simulation import RobotSimulation
from forest_pledge import ForestPledgeAgent
from levy_walk import LevyWalkAgent
from maze_generator import plot_recursive_lines

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Abstract Forest Simulation")
    parser.add_argument("--manual", type=str, default="False",
                        choices=["True", "False"],
                        help="Manual control of the robot")
    parser.add_argument("--alg", type=str, default="forest_pledge",
                        choices=["levy_walk", "forest_pledge"],
                        help="Algorithm to run: 'manual' for user control or 'forest_pledge' for autonomous control.")
    parser.add_argument("--env_seed", type=int, default=2, help="Seed for environment generation")
    parser.add_argument("--noise_seed", type=int, default=2025, help="Seed for noise generation")
    args = parser.parse_args()

    env_seed = args.env_seed
    noise_seed = args.noise_seed

    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
    customized_maze = os.path.join(ROOT_DIR, "customized_maze.png")

    # Generate maze environment.
    random.seed(env_seed)
    np.random.seed(env_seed)
    plot_recursive_lines()
    plt.savefig(customized_maze)

    print("==========================================")
    if args.alg == "manual":
        print("Manual mode: Use arrow keys to control the robot")
    else:
        print("Forest Pledge mode: Running autonomous algorithm")
    print("==========================================")

    # Set the seeds for noise generation.
    random.seed(noise_seed)
    np.random.seed(noise_seed)

    # Choose simulation mode based on the --alg argument.
    if args.manual == "True":
        sim = RobotSimulation(image_path=customized_maze, render=True, manual=True)
        sim.run()
    else:
        sim = RobotSimulation(image_path=customized_maze, render=True, manual=False)
        alg = args.alg
        if alg == "levy_walk":
            agent = LevyWalkAgent(sim)
        else:
            agent = ForestPledgeAgent(sim)
        agent.run()

    # Clean up the generated maze image.
    os.remove(customized_maze)