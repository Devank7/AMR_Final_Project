import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import math

# --- Constants ---
# Screen dimensions
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GRAY = (200, 200, 200)

# Simulation parameters
TOTAL_TIME = 20  # seconds
DT = 0.01  # time step size (0.01s for 100Hz)

# Robot parameters
ROBOT_WIDTH = 50  # pixels, width of the square body
WHEEL_RADIUS = 10 # pixels
WHEEL_DISTANCE = 60 # pixels, distance between the two wheels (L)


class Robot:
    """
    Represents a two-wheeled differential drive robot.
    """
    def __init__(self, x, y, theta, width, wheel_radius, wheel_dist):
        """
        Initializes the robot.
        :param x: Initial x-position
        :param y: Initial y-position
        :param theta: Initial orientation in radians
        :param width: Width of the robot's square body
        :param wheel_radius: Radius of the wheels
        :param wheel_dist: Distance between the centers of the two wheels
        """
        self.x = x
        self.y = y
        self.theta = theta  # Orientation in radians
        
        # Robot dimensions
        self.width = width
        self.wheel_radius = wheel_radius
        self.wheel_dist = wheel_dist
        
        # Create a surface for the robot's appearance to handle rotation
        # We draw the robot once and then rotate this surface
        self.robot_surf = pygame.Surface((width + wheel_radius * 2, width), pygame.SRCALPHA)
        self.draw_robot_shape()

    def draw_robot_shape(self):
        """Draws the robot's body and wheels on its dedicated surface."""
        # Body (centered on the surface)
        body_rect = pygame.Rect(self.wheel_radius, 0, self.width, self.width)
        pygame.draw.rect(self.robot_surf, BLUE, body_rect)
        
        # Wheels
        # Left wheel
        pygame.draw.circle(self.robot_surf, RED, (self.wheel_radius, 0), self.wheel_radius)
        # Right wheel
        pygame.draw.circle(self.robot_surf, RED, (self.wheel_radius, self.width), self.wheel_radius)
        
        # Direction indicator (a line from the center pointing forward)
        center_x = self.robot_surf.get_width() / 2 + 5 # Adjusted for body pos
        center_y = self.robot_surf.get_height() / 2
        pygame.draw.line(self.robot_surf, GREEN, (center_x, center_y), (center_x + self.width / 2, center_y), 3)

    def update(self, v, w, dt):
        """
        Updates the robot's state based on kinematics.
        :param v: Linear velocity (m/s or pixels/s)
        :param w: Angular velocity (rad/s)
        :param dt: Time step (s)
        """
        # Update orientation
        self.theta += w * dt
        
        # Update position
        # Note: We subtract from y because Pygame's y-axis is inverted (0 is at the top)
        # This makes the robot's movement align with standard mathematical coordinates
        # where a positive angle theta=pi/2 means moving "up".
        self.x += v * math.cos(self.theta) * dt
        self.y -= v * math.sin(self.theta) * dt

    def draw(self, screen):
        """
        Draws the robot on the main screen, handling rotation.
        :param screen: The pygame screen surface to draw on.
        """
        # Pygame's rotate is counter-clockwise, so we negate the angle
        # after converting from radians to degrees to match our coordinate system.
        rotated_surf = pygame.transform.rotate(self.robot_surf, math.degrees(self.theta))
        
        # The new rect's center must be the robot's position
        new_rect = rotated_surf.get_rect(center=(self.x, self.y))
        
        screen.blit(rotated_surf, new_rect.topleft)

def draw_axes_and_grid(screen, font):
    """Draws the X-Y axes, grid lines, and labels."""
    origin_x, origin_y = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2
    grid_spacing = 50
    
    # Draw grid lines
    for x in range(0, SCREEN_WIDTH, grid_spacing):
        pygame.draw.line(screen, GRAY, (x, 0), (x, SCREEN_HEIGHT))
    for y in range(0, SCREEN_HEIGHT, grid_spacing):
        pygame.draw.line(screen, GRAY, (0, y), (SCREEN_WIDTH, y))

    # Draw main axes
    pygame.draw.line(screen, BLACK, (origin_x, 0), (origin_x, SCREEN_HEIGHT), 2)
    pygame.draw.line(screen, BLACK, (0, origin_y), (SCREEN_WIDTH, origin_y), 2)

    # Draw axis labels and ticks
    for i in range(0, SCREEN_WIDTH, grid_spacing):
        if i != origin_x:
            label = font.render(str(i - origin_x), True, BLACK)
            screen.blit(label, (i - label.get_width()//2, origin_y + 5))
    for i in range(0, SCREEN_HEIGHT, grid_spacing):
        if i != origin_y:
            # Invert label for y-axis to match coordinate system
            label = font.render(str(origin_y - i), True, BLACK)
            screen.blit(label, (origin_x + 5, i - label.get_height()//2))

def main():
    """
    Main function to run the simulation.
    """
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Test Case 2 - v constant & w constant")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24) # Smaller font for axis labels
    info_font = pygame.font.Font(None, 30)

    # --- Test Case 2 Setup ---
    # Constant linear and angular velocities, resulting in circular motion.
    V_ROBOT = 50  # pixels per second
    W_ROBOT = 0.5  # radians per second. Positive value = counter-clockwise turn.
    
    # Initialize robot at the center (origin), facing right
    origin_x, origin_y = SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2
    initial_theta = 0.0 # Facing right along the positive x-axis
    robot = Robot(origin_x, origin_y, initial_theta, ROBOT_WIDTH, WHEEL_RADIUS, WHEEL_DISTANCE)

    current_time = 0.0
    running = True

    while running and current_time < TOTAL_TIME:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # --- Update Simulation ---
        robot.update(V_ROBOT, W_ROBOT, DT)
        current_time += DT

        # --- Drawing ---
        screen.fill(WHITE)
        draw_axes_and_grid(screen, font)
        robot.draw(screen)

        # --- Display Info ---
        # Calculate position relative to the origin for display
        display_x = robot.x - origin_x
        display_y = origin_y - robot.y
        
        time_text = info_font.render(f"Time: {current_time:.2f}s", True, BLACK)
        pos_text = info_font.render(f"Pos: ({display_x:.1f}, {display_y:.1f})", True, BLACK)
        theta_text = info_font.render(f"Theta: {math.degrees(robot.theta):.1f}°", True, BLACK)
        
        screen.blit(time_text, (10, 10))
        screen.blit(pos_text, (10, 40))
        screen.blit(theta_text, (10, 70))

        pygame.display.flip()

        clock.tick(1 / DT)

    pygame.quit()

if __name__ == "__main__":
    main()