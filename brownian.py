import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

def brownian():
    """Simulate Brownina motion"""
    # Define parameters
    arena_size = 100
    initial_position = np.array([arena_size / 2, arena_size / 2])
    speed = 1
    direction = np.random.rand() * 2 * np.pi
    num_steps = 1000
    positions = [initial_position]
    robot_radius = 2

    def direction_change(new_position, direction):
        """Helper function for direction change after collision"""
        if new_position[0] <= robot_radius or new_position[0] >= arena_size - robot_radius:
            return np.pi - direction
        if new_position[1] <= robot_radius or new_position[1] >= arena_size - robot_radius:
            return -direction
        return direction

    for _ in range(num_steps):
        movement = np.array([np.cos(direction), np.sin(direction)]) * speed
        new_position = positions[-1] + movement
        
        if not (robot_radius <= new_position[0] <= arena_size - robot_radius) or not (robot_radius <= new_position[1] <= arena_size - robot_radius):
            direction = direction_change(new_position, direction)
            direction %= 2 * np.pi
            movement = np.array([np.cos(direction), np.sin(direction)]) * speed
            new_position = positions[-1] + movement
        
        positions.append(new_position)

    positions = np.array(positions)
    fig, ax = plt.subplots()
    ax.set_xlim(0, arena_size)
    ax.set_ylim(0, arena_size)
    robot_circle = Circle(initial_position, robot_radius, color='blue')
    ax.add_patch(robot_circle)

    def init():
        """init function for animation"""
        robot_circle.center = initial_position
        return robot_circle,

    def animate(i):
        """animate function"""
        robot_circle.center = positions[i]
        return robot_circle,

    ani = FuncAnimation(fig, animate, frames=num_steps, init_func=init, blit=True, interval=20)
    plt.show()

# Run the simulation
brownian()
