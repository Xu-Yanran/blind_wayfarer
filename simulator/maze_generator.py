import matplotlib.pyplot as plt
import numpy as np
import random

angle_threshold = 10
triggered_color_pos = '#000001' # (0, 0, 1)
triggered_color_neg = '#000002' # (0, 0, 2)
# triggered_color_pos = "#f70213" # (247, 2, 19) for test
# triggered_color_neg = "#0207f7" # (2, 7, 247) for test

def draw_line_with_hole(x_start, y_start, x_end, y_end, hole_position=None, 
                        min_hole_size=5, max_hole_size=20, triggered_flag=0):
    """Draw a line with a gap (hole) in it."""
    # Calculate the direction vector
    length = np.hypot(x_end - x_start, y_end - y_start)
    direction_x = (x_end - x_start) / length
    direction_y = (y_end - y_start) / length

    # Determine the position of the hole
    if hole_position is None:
        hole_position = np.random.uniform(0.3, 0.7)  # Hole somewhere in the middle section
        hole_size = np.random.uniform(min_hole_size, max_hole_size)

    # Calculate the start and end of the hole
    hole_start_x = x_start + direction_x * (hole_position * length - hole_size / 2)
    hole_start_y = y_start + direction_y * (hole_position * length - hole_size / 2)
    hole_end_x = x_start + direction_x * (hole_position * length + hole_size / 2)
    hole_end_y = y_start + direction_y * (hole_position * length + hole_size / 2)

    
    if triggered_flag == 1:
        # Draw the first segment before the hole
        plt.plot([x_start, hole_start_x], [y_start, hole_start_y], triggered_color_pos, lw=5)
        # Draw the second segment after the hole
        plt.plot([hole_end_x, x_end], [hole_end_y, y_end], triggered_color_pos, lw=5)
    elif triggered_flag == -1:
        # Draw the first segment before the hole
        plt.plot([x_start, hole_start_x], [y_start, hole_start_y], triggered_color_neg, lw=5)
        # Draw the second segment after the hole
        plt.plot([hole_end_x, x_end], [hole_end_y, y_end], triggered_color_neg, lw=5)
    else:
        # Draw the first segment before the hole
        plt.plot([x_start, hole_start_x], [y_start, hole_start_y], 'k-', lw=5)
        # Draw the second segment after the hole
        plt.plot([hole_end_x, x_end], [hole_end_y, y_end], 'k-', lw=5)
    plt.plot([40, 60], [19, 19], 'w-', lw=25)

def recursive_division(x_start, x_end, y_start, y_end, min_size=10):
    """Recursively divide the space and draw lines with small angles and holes."""
    if x_end - x_start < min_size or y_end - y_start < min_size:
        return

    # Randomly decide the orientation: horizontal or vertical
    horizontal = np.random.choice([True, False])

    if horizontal:
        # Horizontal line with a random angle
        angle = np.random.uniform(-20, 20)
        angle_rad = np.deg2rad(angle)

        center_y = np.random.uniform(y_start + min_size, y_end - min_size)
        half_length = (x_end - x_start) / 2

        x_start_line = x_start
        x_end_line = x_end
        y_start_line = center_y - np.tan(angle_rad) * half_length
        y_end_line = center_y + np.tan(angle_rad) * half_length

        draw_line_with_hole(x_start_line, y_start_line, x_end_line, y_end_line)

        # Recursively divide the top and bottom areas
        recursive_division(x_start, x_end, y_start, center_y, min_size)
        recursive_division(x_start, x_end, center_y, y_end, min_size)
    else:
        # Vertical line with a random angle
        angle = np.random.uniform(-20, 20)
        angle_rad = np.deg2rad(angle)

        center_x = np.random.uniform(x_start + min_size, x_end - min_size)
        half_length = (y_end - y_start) / 2

        y_start_line = y_start
        y_end_line = y_end
        x_start_line = center_x - np.tan(angle_rad) * half_length
        x_end_line = center_x + np.tan(angle_rad) * half_length

        if angle > 0:
            draw_line_with_hole(x_start_line, y_start_line, x_end_line, y_end_line, triggered_flag=1)
        else:
            draw_line_with_hole(x_start_line, y_start_line, x_end_line, y_end_line, triggered_flag=-1)

        # Recursively divide the left and right areas
        recursive_division(x_start, center_x, y_start, y_end, min_size)
        recursive_division(center_x, x_end, y_start, y_end, min_size)

def plot_recursive_lines(size=25):
    """Set up the plot and initiate recursive division."""
    plt.figure(figsize=(10, 10))
    plt.plot([40, 60], [20, 20], 'w-', lw=50)
    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.axis('off')

    # Start recursive division
    recursive_division(0, 100, 0, 100, min_size=size)

if __name__ == '__main__':
    # original_size = 25
    # seed_num = 2
    seed_num = 10
    random.seed(seed_num)
    np.random.seed(seed_num)
    plot_recursive_lines(size=25)
    # plt.show()
    plt.savefig(f"/home/abner/simulation-env/temp_maze.png")

    # # Example Usage
    # for i in range(100):
    #     random.seed(i)
    #     np.random.seed(i)
    #     plot_recursive_lines(size=25)
    #     plt.savefig(f"/home/abner/simulation-env/sim_env_layout/maze_{i}.png")
    #     plt.close()

