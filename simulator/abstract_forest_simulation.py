import pygame
import random
import math
import time 
import numpy as np

triggered_color_pos = (0, 0, 1) # (0, 0, 1)
triggered_color_neg = (0, 0, 2) # (0, 0, 2)
# triggered_color_pos = (247, 2, 19) # (247, 2, 19) for visualization
# triggered_color_neg = (2, 7, 247) # (2, 7, 247) for visualization

class RobotSimulation:
    def __init__(self, num_trees=20, 
                 robot_starting_point=(500, 750),
                 image_path = "demo_maze.png",
                 width=1000, height=1000,
                 noise_factor = 0.2,
                 compass_noise = 1.0,
                render = False,
                manual = False,
                visualize_obs_orientation = False
                 ):
        
        self.render = render
        self.manual = manual
        pygame.init()
        
        self.turning_degree = 30
        self.recovery_angle = 5
        self.robot_starting_point = robot_starting_point

        self.WIDTH, self.HEIGHT = width, height
        if self.render:
            self.win = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
            pygame.display.set_caption("2D Robot Simulation with Maze")

        self.num_trees = num_trees

        self.clock = pygame.time.Clock()

        self.noise_factor = noise_factor
        self.compass_noise = compass_noise
        
        self.grid_size = 10 
        self.grid = {}
        self.grid_pos = {}
        self.grid_neg = {}

        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.BLUE = (0, 0, 153)
        self.GREEN = (0, 102, 0)
        self.RED = (153, 0, 0)

        # Monsterborg
        # self.robot_width = 4
        # self.robot_height = 4

        # Forest Rover
        self.robot_width = 8
        self.robot_height = 8

        self.robot_x, self.robot_y = robot_starting_point[0], robot_starting_point[1]
        self.robot_angle = 90
        self.robot_speed = 1
        
        self.trajectory = []

        self.distance_traveled = 0
        # The desired mean value is 2m, 200px
        self.dist_trigger = self.neg_exp(200)
        self.dist_for_prob = 0
        self.angle_counter = 0

        self.waypoint_flag = True
        self.state = 0

        self.corners = []

        self.loops = 0

        self.obstacles = set()
        self.obstacles_pos = set()
        self.obstacles_neg = set()

        image = pygame.image.load(image_path)
        self.walls = self.create_obstacles(image)
        self.random_obstacles = []

        self.prob_stuck = []

        self.path_length = 0
        self.lost_flag = False
        self.stuck_flag = False

        self.start_time = time.time()

        self.starting_point = (self.robot_x, self.robot_y)
        self.desired_orientation = self.robot_angle

        self.generate_standing_trees()

        # for auto_correlation
        self.yaw_history = []
        self.yaw_window_size = 1000
        self.coef_auto_corr = 0.0
        self.change_flag = False

        # for random_insist, collision_vector_pledge
        self.turning_direction = 1

        # for stepback_steerturn
        self.collision_vector = "None"

        # for levy walk
        self.step_length = 0
        self.turn_done = False
        self.move_done = False
        self.desired_angle = 0

        # for pledge-inspired
        self.step_size = 12

    def neg_exp(self, desired_mean):
        return 10 + random.expovariate(1/desired_mean)

    def create_obstacles(self, image):
        for x in range(self.WIDTH):
            for y in range(self.HEIGHT):
                color = image.get_at((x, y))
                # Check if the pixel is not white
                if color != (255, 255, 255, 255):  # (R, G, B, A)
                    rgb_color = (color[0], color[1], color[2])
                    if rgb_color == triggered_color_neg:
                        self.obstacles_neg.add((x, y))
                        self.add_to_grid((x, y), flag=-1)
                    elif rgb_color == triggered_color_pos:
                        self.obstacles_pos.add((x, y))
                        self.add_to_grid((x, y), flag=1)
                    elif rgb_color == (0, 0, 0):
                        self.obstacles.add((x, y))
                        self.add_to_grid((x, y), flag=0)
        return self.obstacles

    def is_obstacle(self, position):
        grid_x = int(position[0]) // self.grid_size
        grid_y = int(position[1]) // self.grid_size
        if (grid_x, grid_y) in self.grid:
            for obstacle in self.grid[(grid_x, grid_y)]:
                if obstacle == (int(position[0]), int(position[1])):
                    return [True, 0]
        if (grid_x, grid_y) in self.grid_pos:
            for obstacle in self.grid_pos[(grid_x, grid_y)]:
                if obstacle == (int(position[0]), int(position[1])):
                    return [True, 1]
        if (grid_x, grid_y) in self.grid_neg:
            for obstacle in self.grid_neg[(grid_x, grid_y)]:
                if obstacle == (int(position[0]), int(position[1])):
                    return [True, -1]
        return [False, 0]
    
    def add_to_grid(self, pos, flag=0):
        grid_x = pos[0] // self.grid_size
        grid_y = pos[1] // self.grid_size
        if flag == 0:
            if (grid_x, grid_y) not in self.grid:
                self.grid[(grid_x, grid_y)] = []
            self.grid[(grid_x, grid_y)].append(pos)
        elif flag == 1:
            if (grid_x, grid_y) not in self.grid_pos:
                self.grid_pos[(grid_x, grid_y)] = []
            self.grid_pos[(grid_x, grid_y)].append(pos)
        else:
            if (grid_x, grid_y) not in self.grid_neg:
                self.grid_neg[(grid_x, grid_y)] = []
            self.grid_neg[(grid_x, grid_y)].append(pos)

    def get_compass_angle(self, angle):
        if len(self.yaw_history) > self.yaw_window_size:
            self.yaw_history.pop(0)
        angle_with_noise = np.random.normal(angle, self.compass_noise)

        # To calculate the difference between rover_orientation and desired_direction
        # diff_angle_with_noise = get_angle_diff(self.desired_orientation, angle_with_noise)
        # End calculation

        # Append raw yaw
        self.yaw_history.append(angle_with_noise)
        # Append yaw difference - [-180, 180]
        return angle_with_noise

    def get_run_time(self):
        return time.time() - self.start_time

    def get_path_length(self):
        return int(self.path_length)
    
    def get_lost_flag(self):
        return self.lost_flag
    
    def get_stuck_flag(self):
        return self.stuck_flag

    def generate_standing_trees(self):
        tree_radius = 5  # Radius of the tree circle
        for _ in range(self.num_trees):
            while True:
                x = random.randint(tree_radius, self.WIDTH - tree_radius)
                y = random.randint(tree_radius, self.HEIGHT - tree_radius)

                # Check if the position is within the robot starting point area
                r_x = self.robot_starting_point[0]
                r_y = self.robot_starting_point[1]
                if (r_x - 30 <= x <= r_x + 30 and r_y - 30 <= y <= r_y + 30):
                    continue

                # Check if the position is an obstacle (either a wall or another obstacle)
                is_valid = True
                for dx in range(-tree_radius, tree_radius + 1):
                    for dy in range(-tree_radius, tree_radius + 1):
                        if math.sqrt(dx ** 2 + dy ** 2) <= tree_radius:
                            if self.is_obstacle((x + dx, y + dy))[0]:
                                is_valid = False
                                break
                    if not is_valid:
                        break
                    
                # Check against other random obstacles, considering their radius
                if is_valid:
                    for obs in self.random_obstacles:
                        distance = math.sqrt((obs[0][0] - x) ** 2 + (obs[0][1] - y) ** 2)
                        if distance < obs[1] + tree_radius:  # Tree radius + obstacle radius
                            is_valid = False
                            break

                if is_valid:
                    self.random_obstacles.append([[x, y], tree_radius, (34, 139, 34)])
                    break


    def draw_robot(self, x, y, angle):
        half_width = self.robot_width / 2
        half_height = self.robot_height / 2
        rad_angle = math.radians(angle)
        
        cos_a = math.cos(rad_angle)
        sin_a = math.sin(rad_angle)
        
        self.corners = [
            (x + half_width * cos_a - half_height * sin_a, y - half_width * sin_a - half_height * cos_a),
            (x - half_width * cos_a - half_height * sin_a, y + half_width * sin_a - half_height * cos_a),
            (x - half_width * cos_a + half_height * sin_a, y + half_width * sin_a + half_height * cos_a),
            (x + half_width * cos_a + half_height * sin_a, y - half_width * sin_a + half_height * cos_a)
        ]
        
        pygame.draw.polygon(self.win, self.BLUE, self.corners)
        
        front_x = x + half_width * cos_a
        front_y = y - half_width * sin_a
        pygame.draw.line(self.win, (255,215,0), (x, y), (front_x, front_y), 2)
        
    def draw_prob_stuck(self):
        for stuck in self.prob_stuck:
            pygame.draw.circle(self.win, self.RED, (int(stuck[0]), int(stuck[1])), 3)
    
    def draw_walls(self):
        for obstacle in self.obstacles:
            self.win.set_at(obstacle, self.BLACK)
        for obstacle in self.obstacles_pos:
            self.win.set_at(obstacle, triggered_color_pos)
        for obstacle in self.obstacles_neg:
            self.win.set_at(obstacle, triggered_color_neg)

    def draw_random_obstacles(self):
        for obs in self.random_obstacles:
            pygame.draw.circle(self.win, obs[2], obs[0], obs[1])
            
    def detect_collision(self, x, y, angle):
        half_width = self.robot_width / 2
        half_height = self.robot_height / 2
        rad_angle = math.radians(angle)
    
        cos_a = math.cos(rad_angle)
        sin_a = math.sin(rad_angle)
    
        corners = [
            (x + half_width * cos_a - half_height * sin_a, y - half_width * sin_a - half_height * cos_a),
            (x - half_width * cos_a - half_height * sin_a, y + half_width * sin_a - half_height * cos_a),
            (x - half_width * cos_a + half_height * sin_a, y + half_width * sin_a + half_height * cos_a),
            (x + half_width * cos_a + half_height * sin_a, y - half_width * sin_a + half_height * cos_a)
        ]

        left_side = [corners[0], corners[1]]  # Front-Left and Back-Left corners
        right_side = [corners[3], corners[2]]  # Front-Right and Back-Right corners

        left_collision = False
        right_collision = False

        obstacle_ori_sign = 0

        # Check if any corner collides
        for i, corner in enumerate(corners):
            collision_check = self.is_obstacle(corner)
            obstacle_ori_sign = collision_check[1]
            if collision_check[0]:
                if i in [3]:  # Right side corners
                    right_collision = True
                elif i in [0]:  # Left side corners
                    left_collision = True
                break

        # Check if both front-left and front-right corners have collisions
        if left_collision and right_collision:
            return ["front", obstacle_ori_sign]
        elif left_collision:
            return ["left", obstacle_ori_sign]
        elif right_collision:
            return ["right", obstacle_ori_sign]

        # Check points along each edge
        num_samples = 5  # Reduced number of points to sample along each edge
        for i in range(len(corners)):
            start_corner = corners[i]
            end_corner = corners[(i + 1) % len(corners)]
            for j in range(num_samples):
                sample_point = (
                    start_corner[0] + (end_corner[0] - start_corner[0]) * j / num_samples,
                    start_corner[1] + (end_corner[1] - start_corner[1]) * j / num_samples
                )
                collision_check = self.is_obstacle(sample_point)
                obstacle_ori_sign = collision_check[1]
                if collision_check[0]:
                    if i in [3] or (i == 3 and j > num_samples // 2):  # Right side edges
                        right_collision = True
                    elif i in [0] or (i == 0 and j > num_samples // 2):  # Left side edges
                        left_collision = True
                    break

        # Check if both front-left and front-right corners have collisions
        if left_collision and right_collision:
            return ["front", obstacle_ori_sign]
        elif left_collision:
            return ["left", obstacle_ori_sign]
        elif right_collision:
            return ["right", obstacle_ori_sign]

        for obs in self.random_obstacles:
            if math.sqrt((obs[0][0] - x) ** 2 + (obs[0][1] - y) ** 2) < obs[1] + self.robot_height/2:
                dist_obs = obs[1] + self.robot_height/2
                dist_left_corner = math.sqrt((obs[0][0] - corners[0][0]) ** 2 + (obs[0][1] - corners[0][1]) ** 2)
                dist_right_corner = math.sqrt((obs[0][0] - corners[3][0]) ** 2 + (obs[0][1] - corners[3][1]) ** 2)
                if dist_left_corner < dist_obs and dist_right_corner < dist_obs:
                    return ["front", 0]
                elif dist_left_corner < dist_obs:
                    return ["left", 0]
                elif dist_right_corner < dist_obs:
                    return ["right", 0]
                    
        return ["None", 0]

    def move_forward(self):
        noise = np.random.normal(0, self.noise_factor)
        move_step = self.robot_speed

        for step in range(move_step, 0, -1):
            new_x = self.robot_x + step * math.cos(noise + math.radians(self.robot_angle))
            new_y = self.robot_y - step * math.sin(noise + math.radians(self.robot_angle))
            if self.detect_collision(new_x, new_y, self.robot_angle)[0] == "None":
                self.distance_traveled += step
                self.dist_for_prob += step
                self.path_length += step
                self.robot_x, self.robot_y = new_x, new_y
                break

    def move_backward(self):
        noise = np.random.normal(0, self.noise_factor)
        move_step = 3 * self.robot_speed

        for step in range(move_step, 0, -1):
            new_x = self.robot_x - step * math.cos(noise + math.radians(self.robot_angle))
            new_y = self.robot_y + step * math.sin(noise + math.radians(self.robot_angle))
            if self.detect_collision(new_x, new_y, self.robot_angle)[0] == "None":
                self.robot_x, self.robot_y = new_x, new_y
                self.path_length += step
                break

    def rotate_right(self, wp_flag=False):
        if wp_flag:
            new_angle = (self.robot_angle - 2) % 360
            if self.detect_collision(self.robot_x, self.robot_y, new_angle)[0] == "None":
                self.robot_angle = new_angle
        else:
            new_angle = (self.robot_angle - self.turning_degree) % 360
            if self.detect_collision(self.robot_x, self.robot_y, new_angle)[0] == "None":
                self.angle_counter += self.turning_degree
                self.robot_angle = new_angle

    def rotate_left(self, wp_flag=False):
        if wp_flag:
            new_angle = (self.robot_angle + 2) % 360
            if self.detect_collision(self.robot_x, self.robot_y, new_angle)[0] == "None":
                self.robot_angle = new_angle
        else:
            new_angle = (self.robot_angle + self.turning_degree) % 360
            if self.detect_collision(self.robot_x, self.robot_y, new_angle)[0] == "None":
                self.angle_counter -= self.turning_degree
                self.robot_angle = new_angle
    
    def recovery_left(self):
        new_angle = (self.robot_angle + self.recovery_angle) % 360
        if self.detect_collision(self.robot_x, self.robot_y, new_angle)[0] == "None":
            self.angle_counter -= self.recovery_angle
            self.robot_angle = new_angle

    def recovery_right(self):
        new_angle = (self.robot_angle - self.recovery_angle) % 360
        if self.detect_collision(self.robot_x, self.robot_y, new_angle)[0] == "None":
            self.angle_counter += self.recovery_angle
            self.robot_angle = new_angle

    def collision_check(self):
        collision_detected = self.detect_collision(
            self.robot_x + self.robot_speed * math.cos(math.radians(self.robot_angle)),
            self.robot_y - self.robot_speed * math.sin(math.radians(self.robot_angle)),
            self.robot_angle
        )
        self.robot_angle = self.get_compass_angle(self.robot_angle)

        # Update trajectory
        self.trajectory.append((self.robot_x, self.robot_y))

        # Use probability of stuck 
        if self.dist_for_prob >= self.dist_trigger:
            collision_detected[0] = "front"
            self.prob_stuck.append([self.robot_x, self.robot_y, self.robot_angle])
            self.dist_trigger = self.neg_exp(200)
            self.dist_for_prob = 0
        return collision_detected

    def handle_key(self):
        keys = pygame.key.get_pressed()
            
        new_x, new_y = self.robot_x, self.robot_y
        new_angle = self.robot_angle
        if keys[pygame.K_LEFT]:
            new_angle = (self.robot_angle + 5) % 360
        if keys[pygame.K_RIGHT]:
            new_angle = (self.robot_angle - 5) % 360
        if keys[pygame.K_UP]:
            new_x += self.robot_speed * math.cos(math.radians(self.robot_angle))
            new_y -= self.robot_speed * math.sin(math.radians(self.robot_angle))
        if keys[pygame.K_DOWN]:
            new_x -= self.robot_speed * math.cos(math.radians(self.robot_angle))
            new_y += self.robot_speed * math.sin(math.radians(self.robot_angle))
            
        if self.detect_collision(new_x, new_y, new_angle)[0] == "None":
            self.robot_x, self.robot_y, self.robot_angle = new_x, new_y, new_angle
        else:
            print("Collision detected")

    def draw_arrow(self, surface, color, start, end, arrow_width=15, arrow_length=45, alpha_value=100):
        """
        Draws a semi-transparent arrow from `start` to `end` on the given surface.
        """
        # Create a transparent surface
        arrow_surface = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
        arrow_surface.fill((0, 0, 0, 0))  # Fully transparent background

        # Convert color to RGBA with transparency
        color_with_alpha = (*color, alpha_value)

        # Draw semi-transparent arrow line
        pygame.draw.line(arrow_surface, color_with_alpha, start, end, arrow_width)

        # Calculate arrowhead angles
        angle = pygame.math.Vector2(end[0] - start[0], end[1] - start[1]).angle_to((1, 0))
        arrow_tip1 = (
            end[0] + arrow_length * pygame.math.Vector2(1, 0).rotate(angle + 135).x,
            end[1] - arrow_length * pygame.math.Vector2(1, 0).rotate(angle + 135).y
        )
        arrow_tip2 = (
            end[0] + arrow_length * pygame.math.Vector2(1, 0).rotate(angle - 135).x,
            end[1] - arrow_length * pygame.math.Vector2(1, 0).rotate(angle - 135).y
        )

        # Draw semi-transparent arrowhead
        end[1] -= 10
        pygame.draw.polygon(arrow_surface, color_with_alpha, [end, arrow_tip1, arrow_tip2])

        # Blit the arrow surface onto the main surface
        surface.blit(arrow_surface, (0, 0))

    def draw_starting_point(self, x_center, y_center):
        num_points = 5
        outer_radius = 20

        angle_step = 360.0 / (2 * num_points)
        inner_radius = outer_radius * 0.5

        points = []
        for i in range(2 * num_points):
            if i % 2 == 0:
                r = outer_radius
            else:
                r = inner_radius
            
            angle_deg = i * angle_step
            angle_rad = math.radians(angle_deg)
            
            x = x_center + r * math.cos(angle_rad)
            y = y_center + r * math.sin(angle_rad)
            points.append((x, y))
        
        # Create a transparent surface
        star_surface = pygame.Surface((outer_radius * 2, outer_radius * 2), pygame.SRCALPHA)
        star_surface.fill((0, 0, 0, 0))  # Fully transparent background

        # Draw star with RGBA color (Red with 128 alpha)
        pygame.draw.polygon(star_surface, (255, 0, 0, 100), 
                             [(x - x_center + outer_radius, y - y_center + outer_radius) for x, y in points])

        # Blit the star onto the main window
        self.win.blit(star_surface, (x_center - outer_radius, y_center - outer_radius))

    def draw_destination(self):
        length = 125
        pygame.draw.line(self.win, self.BLACK, (0, 20), (self.WIDTH / 2 - length, 20), 4)
        pygame.draw.line(self.win, self.BLACK, (self.WIDTH / 2 + length, 20), (self.WIDTH, 20), 4)

        font = pygame.font.SysFont(None, 57)
        text_surface = font.render("Destination", True, self.BLACK)
        text_rect = text_surface.get_rect(center=(self.WIDTH / 2, 20))

        self.win.blit(text_surface, text_rect)

    def get_remaining_distance(self):
        return int(self.robot_y)

    def _render(self):
        self.win.fill(self.WHITE)
        self.draw_walls()
        self.draw_random_obstacles()
        self.draw_destination()

        if len(self.trajectory) > 1:
            pygame.draw.lines(self.win, self.GREEN, False, self.trajectory, width=2)
            
        self.draw_robot(self.robot_x, self.robot_y, self.robot_angle)

        self.draw_starting_point(self.starting_point[0], self.starting_point[1])
        self.draw_arrow(self.win, self.BLACK, [75, 250], [75, 150])

        self.draw_prob_stuck()

        pygame.draw.line(self.win, self.BLACK, (0, 0), (self.WIDTH, 0), 2)
        pygame.draw.line(self.win, self.BLACK, (0, self.HEIGHT-2), (self.WIDTH, self.HEIGHT-2), 2)
        pygame.draw.line(self.win, self.BLACK, (0, 0), (0, self.HEIGHT), 2)
        pygame.draw.line(self.win, self.BLACK, (self.WIDTH-2, 0), (self.WIDTH-2, self.HEIGHT), 2)

        pygame.display.update()

    def run(self):
        running = True
        while running:
            self.loops += 1

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            
            if self.manual:
                self.handle_key()

            self.trajectory.append((self.robot_x, self.robot_y))

            if self.render:
                self._render()

            if self.robot_y < 0:
                running = False
                print("Reached the goal")
            if self.robot_x < 0 or self.robot_x > self.WIDTH or self.robot_y > self.HEIGHT:
                running = False
                self.lost_flag = True
            if self.get_run_time() > 90 or self.get_path_length()> 20000:
                running = False
                self.stuck_flag = True


if __name__ == "__main__":
    import os
    import argparse
    from maze_generator import plot_recursive_lines
    from matplotlib import pyplot as plt    

    # Read parameters from command line
    parser = argparse.ArgumentParser(description="Abstract Forest Simulation")
    parser.add_argument("--env_seed", type=int, default=2, help="Seed for environment generation")
    parser.add_argument("--noise_seed", type=int, default=2025, help="Seed for noise generation")
    args = parser.parse_args()
    
    env_seed = args.env_seed
    noise_seed = args.noise_seed

    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
    customized_maze = os.path.join(ROOT_DIR, "customized_maze.png")

    # generate maze environment
    random.seed(env_seed)
    np.random.seed(env_seed)
    plot_recursive_lines()
    plt.savefig(customized_maze)

    print("==========================================")
    print("Use arrow keys to control the robot")
    print("==========================================")

    # Setup simulation
    random.seed(noise_seed)
    np.random.seed(noise_seed)
    sim = RobotSimulation(image_path=customized_maze, render=True, manual=True)

    # Remove the generated maze
    os.remove(customized_maze)
    # Run simulation
    sim.run()