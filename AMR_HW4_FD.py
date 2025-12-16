import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import math
import numpy as np

# --- Constants ---
SCREEN_WIDTH, SCREEN_HEIGHT = 1000, 800
WHITE, BLACK, RED, GREEN, BLUE, GRAY, ORANGE = (255, 255, 255), (0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255), (200, 200, 200), (255, 165, 0)

# --- Simulation Parameters ---
DT = 0.01  # Time step

# Robot Physical Properties (in SI-like units for calculation, pixels for display)
# We'll use kg, meters, etc. for physics, then scale to pixels. 1 meter = 100 pixels.
PX_PER_METER = 100

ROBOT_MASS = 5.0  # kg
# Moment of inertia for a square body: I = m * (w^2 + h^2) / 12
ROBOT_WIDTH_M = 0.5 # meters
ROBOT_IC = ROBOT_MASS * (ROBOT_WIDTH_M**2 + ROBOT_WIDTH_M**2) / 12 # kg*m^2

WHEEL_RADIUS_M = 0.05  # meters (5 cm)
WHEEL_DISTANCE_M = 0.6   # meters (L)

# Convert to pixels for drawing
ROBOT_WIDTH_PX = int(ROBOT_WIDTH_M * PX_PER_METER)
WHEEL_RADIUS_PX = int(WHEEL_RADIUS_M * PX_PER_METER)
WHEEL_DISTANCE_PX = int(WHEEL_DISTANCE_M * PX_PER_METER)

class Robot:
    """Represents a differential drive robot with dynamic properties."""
    def __init__(self, x, y, theta=0):
        # State variables
        self.x, self.y, self.theta = x, y, theta  # Pose (pixels, radians)
        self.v, self.w = 0.0, 0.0  # Velocities (m/s, rad/s)

        # Physical Properties
        self.m = ROBOT_MASS
        self.Ic = ROBOT_IC
        self.r = WHEEL_RADIUS_M
        self.L = WHEEL_DISTANCE_M

        # Drawing surface
        self.robot_surf = pygame.Surface((ROBOT_WIDTH_PX + WHEEL_RADIUS_PX * 2, ROBOT_WIDTH_PX), pygame.SRCALPHA)
        self.draw_robot_shape()
    
    def draw_robot_shape(self):
        body_rect = pygame.Rect(WHEEL_RADIUS_PX, 0, ROBOT_WIDTH_PX, ROBOT_WIDTH_PX)
        pygame.draw.rect(self.robot_surf, BLUE, body_rect)
        pygame.draw.circle(self.robot_surf, RED, (WHEEL_RADIUS_PX, 0), WHEEL_RADIUS_PX)
        pygame.draw.circle(self.robot_surf, RED, (WHEEL_RADIUS_PX, ROBOT_WIDTH_PX), WHEEL_RADIUS_PX)
        center_x = self.robot_surf.get_width() / 2
        center_y = self.robot_surf.get_height() / 2
        pygame.draw.line(self.robot_surf, GREEN, (center_x, center_y), (center_x + ROBOT_WIDTH_PX / 2, center_y), 3)

    def update_dynamics(self, tau_r, tau_l, dt):
        """
        FORWARD DYNAMICS: Updates robot state based on motor torques.
        :param tau_r: Right motor torque (N*m)
        :param tau_l: Left motor torque (N*m)
        :param dt: Time step (s)
        :return: A tuple of (linear_acceleration, angular_acceleration)
        """
        # --- Calculate accelerations from torques (Forward Dynamics Model) ---
        # This is a simplified model ignoring friction and complex wheel dynamics
        v_dot = (tau_r + tau_l) / (self.m * self.r)
        w_dot = (self.L * (tau_r - tau_l)) / (2 * self.Ic * self.r)
        
        # --- Integrate accelerations to get new velocities ---
        self.v += v_dot * dt
        self.w += w_dot * dt

        # --- Integrate velocities to get new pose (Kinematics) ---
        self.theta += self.w * dt
        delta_x = self.v * math.cos(self.theta) * dt
        delta_y = self.v * math.sin(self.theta) * dt
        
        # Convert delta from meters to pixels for updating position
        self.x += delta_x * PX_PER_METER
        self.y -= delta_y * PX_PER_METER # Pygame y-axis is inverted

        return v_dot, w_dot

    def draw(self, screen):
        rotated_surf = pygame.transform.rotate(self.robot_surf, math.degrees(self.theta))
        new_rect = rotated_surf.get_rect(center=(self.x, self.y))
        screen.blit(rotated_surf, new_rect.topleft)

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Forward Dynamics")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 30)

    # --- Setup ---
    origin_x, origin_y = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2
    
    # --- State Machine Setup for Square Path ---
    SIDE_LENGTH_M = 4.0
    
    # Start at a defined corner, facing up
    start_pos_px = (origin_x - 200, origin_y + 200)
    robot = Robot(start_pos_px[0], start_pos_px[1], theta=math.pi/2) 
    
    # State variables
    current_state = 'MOVING_STRAIGHT' # Can be 'MOVING_STRAIGHT' or 'TURNING'
    legs_completed = 0
    start_of_leg_pos = start_pos_px
    target_heading = robot.theta
    path_complete = False
    running = True

    # Torque constants
    FORWARD_TORQUE = 0.4 # N*m to apply when moving straight
    TURN_TORQUE = 0.3    # N*m to apply when turning
    
    # --- Data Logging ---
    log_tau_r, log_tau_l = [], []
    log_v_dot, log_w_dot = [], []
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if path_complete:
            running = False
            continue

        tau_r, tau_l = 0.0, 0.0 # Default to zero torque

        # --- State Machine Logic ---
        if current_state == 'MOVING_STRAIGHT':
            # Apply forward torque
            tau_r, tau_l = FORWARD_TORQUE, FORWARD_TORQUE
            
            dist_traveled = math.hypot(robot.x - start_of_leg_pos[0], robot.y - start_of_leg_pos[1])

            # If we've traveled the side length, switch to turning
            if dist_traveled >= SIDE_LENGTH_M * PX_PER_METER:
                legs_completed += 1
                robot.v = 0 # Stop before turning
                
                if legs_completed >= 4:
                    path_complete = True
                else:
                    current_state = 'TURNING'
                    # Calculate target for a 90-degree right turn
                    target_heading = robot.theta - (math.pi / 2)
                    # Snap to expected corner to prevent drift
                    expected_x = start_of_leg_pos[0] + (SIDE_LENGTH_M * PX_PER_METER * math.cos(robot.theta))
                    expected_y = start_of_leg_pos[1] - (SIDE_LENGTH_M * PX_PER_METER * math.sin(robot.theta))
                    robot.x, robot.y = expected_x, expected_y

        elif current_state == 'TURNING':
            angle_error = target_heading - robot.theta
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi # Normalize error
            
            # If the turn is almost complete, switch back to moving straight
            if abs(angle_error) < 0.05: # ~3 degree tolerance
                current_state = 'MOVING_STRAIGHT'
                robot.theta = target_heading # Snap to the exact heading
                robot.w = 0 # *** FIX: Reset angular velocity after turning ***
                start_of_leg_pos = (robot.x, robot.y) # Set start for the next leg
            else:
                # Apply opposite torques to turn right (clockwise)
                tau_r, tau_l = -TURN_TORQUE, TURN_TORQUE

        v_dot, w_dot = robot.update_dynamics(tau_r, tau_l, DT)

        # Log data
        log_tau_r.append(tau_r)
        log_tau_l.append(tau_l)
        log_v_dot.append(v_dot)
        log_w_dot.append(w_dot)

        # --- Drawing ---
        screen.fill(WHITE)
        robot.draw(screen)

        # --- Display Info ---
        info_texts = [
            f"State: {current_state}",
            f"Legs Completed: {legs_completed} / 4",
            f"Torque R: {tau_r:+.2f} Nm | Torque L: {tau_l:+.2f} Nm",
            f"V_dot: {v_dot:+.2f} m/s^2 | W_dot: {w_dot:+.2f} rad/s^2",
        ]
        for i, text in enumerate(info_texts):
            screen.blit(font.render(text, True, BLACK), (10, 10 + i * 30))

        pygame.display.flip()
        clock.tick(1 / DT)
        
    pygame.quit()
    
    # Post-Simulation Analysis
    
    avg_tau_r_sim = np.mean(log_tau_r)
    avg_tau_l_sim = np.mean(log_tau_l)
    avg_v_dot_sim = np.mean(log_v_dot)
    avg_w_dot_sim = np.mean(log_w_dot)
    
    v_dot_theory = (avg_tau_r_sim + avg_tau_l_sim) / (ROBOT_MASS * WHEEL_RADIUS_M)
    w_dot_theory = (WHEEL_DISTANCE_M * (avg_tau_r_sim - avg_tau_l_sim)) / (2 * ROBOT_IC * WHEEL_RADIUS_M)

    v_dot_error = (abs(avg_v_dot_sim - v_dot_theory) / (abs(v_dot_theory) if v_dot_theory != 0 else 1)) * 100
    w_dot_error = (abs(avg_w_dot_sim - w_dot_theory) / (abs(w_dot_theory) if w_dot_theory != 0 else 1)) * 100

    print(f"Path Completed: 1 Square Loop")
    print(f"Total Data Points: {len(log_tau_r)}\n")
    print(f"{'Parameter':<25} {'Simulation Avg':<15} {'Theoretical':<15} {'Error (%)':<10}")
    print("-"*20)
    print(f"{'Avg Right Torque (Nm)':<25} {avg_tau_r_sim:<15.4f}")
    print(f"{'Avg Left Torque (Nm)':<25} {avg_tau_l_sim:<15.4f}\n")
    print(f"{'Avg Linear Accel (m/s^2)':<25} {avg_v_dot_sim:<15.4f} {v_dot_theory:<15.4f} {v_dot_error:<10.4f}")
    print(f"{'Avg Angular Accel (rad/s^2)':<25} {avg_w_dot_sim:<15.4f} {w_dot_theory:<15.4f} {w_dot_error:<10.4f}")



if __name__ == "__main__":
    main()