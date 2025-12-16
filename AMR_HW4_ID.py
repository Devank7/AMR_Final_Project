import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import math
import numpy as np

# Constants
SCREEN_WIDTH, SCREEN_HEIGHT = 1000, 800
WHITE, BLACK, RED, GREEN, BLUE = (255, 255, 255), (0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255)

# Simulation Parameters
DT = 0.01  # Time step
SIMULATION_DURATION = 15 # seconds

# Robot Physical Properties (in SI-like units for calculation, pixels for display)
PX_PER_METER = 100
ROBOT_MASS = 5.0
ROBOT_WIDTH_M = 0.5
ROBOT_IC = ROBOT_MASS * (ROBOT_WIDTH_M**2 + ROBOT_WIDTH_M**2) / 12
WHEEL_RADIUS_M = 0.05
WHEEL_DISTANCE_M = 0.6

# Convert to pixels for drawing
ROBOT_WIDTH_PX = int(ROBOT_WIDTH_M * PX_PER_METER)
WHEEL_RADIUS_PX = int(WHEEL_RADIUS_M * PX_PER_METER)

class Robot:
    """Represents a differential drive robot with dynamic properties."""
    def __init__(self, x, y, theta=0):
        self.x, self.y, self.theta = x, y, theta
        self.v, self.w = 0.0, 0.0
        self.m, self.Ic, self.r, self.L = ROBOT_MASS, ROBOT_IC, WHEEL_RADIUS_M, WHEEL_DISTANCE_M
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

    def calculate_inverse_dynamics(self, v_dot, w_dot):
        """INVERSE DYNAMICS: Calculates required motor torques for desired accelerations."""
        tau_r = (self.m * self.r * v_dot / 2) + (self.Ic * self.r * w_dot / self.L)
        tau_l = (self.m * self.r * v_dot / 2) - (self.Ic * self.r * w_dot / self.L)
        return tau_r, tau_l

    def update_physics(self, tau_r, tau_l, dt):
        """FORWARD DYNAMICS: Updates robot state based on motor torques."""
        v_dot_actual = (tau_r + tau_l) / (self.m * self.r)
        w_dot_actual = (self.L * (tau_r - tau_l)) / (2 * self.Ic * self.r)
        
        self.v += v_dot_actual * dt
        self.w += w_dot_actual * dt

        self.theta += self.w * dt
        delta_x = self.v * math.cos(self.theta) * dt * PX_PER_METER
        delta_y = self.v * math.sin(self.theta) * dt * PX_PER_METER
        
        self.x += delta_x
        self.y -= delta_y

        return v_dot_actual, w_dot_actual

    def draw(self, screen):
        rotated_surf = pygame.transform.rotate(self.robot_surf, math.degrees(self.theta))
        new_rect = rotated_surf.get_rect(center=(self.x, self.y))
        screen.blit(rotated_surf, new_rect.topleft)

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Inverse Dynamics")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 30)

    # Setup 
    robot = Robot(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2, theta=0) 
    
    # HERE ARE THE CONSTANT INPUT ACCELERATIONS 
    V_DOT_INPUTED = 0.2  # m/s^2
    W_DOT_INPUTED = 0.1  # rad/s^2

    running = True
    sim_time = 0

    log_tau_r, log_tau_l = [], []
    log_v_dot_act, log_w_dot_act = [], []
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        if sim_time >= SIMULATION_DURATION:
            running = False
            continue

        # --- Core Dynamics Calculation ---
        # 1. INVERSE DYNAMICS: Calculate torques from the constant desired accelerations
        tau_r, tau_l = robot.calculate_inverse_dynamics(V_DOT_INPUTED, W_DOT_INPUTED)
        
        # 2. FORWARD DYNAMICS: Simulate what happens when these torques are applied
        v_dot_actual, w_dot_actual = robot.update_physics(tau_r, tau_l, DT)

        # Log Data
        log_tau_r.append(tau_r)
        log_tau_l.append(tau_l)
        log_v_dot_act.append(v_dot_actual)
        log_w_dot_act.append(w_dot_actual)

        # Drawing
        screen.fill(WHITE)
        robot.draw(screen)

        # Display Info
        info_texts = [
            f"Time: {sim_time:.2f}s / {SIMULATION_DURATION}s",
            f"Inputed V_dot: {V_DOT_INPUTED:+.2f} m/s^2",
            f"Inputed W_dot: {W_DOT_INPUTED:+.2f} rad/s^2",
            "---------------------------------",
            f"CALCULATED Torque R: {tau_r:+.2f} Nm",
            f"CALCULATED Torque L: {tau_l:+.2f} Nm",
        ]
        for i, text in enumerate(info_texts):
            screen.blit(font.render(text, True, BLACK), (10, 10 + i * 30))

        pygame.display.flip()
        sim_time += DT
        clock.tick(1 / DT)
        
    pygame.quit()
    
    # Post-Simulation Analysis 
    # The Core Calculation 
    # Since desired accelerations are constant, the required torques are also constant.
    # We can recalculate them here to show the direct input -> output relationship.
    required_tau_r, required_tau_l = robot.calculate_inverse_dynamics(V_DOT_INPUTED, W_DOT_INPUTED)

    print("INPUTS (Desired Motion):")
    print("------------------------")
    print(f"{'Inputed Linear Accel (m/s^2):':<35} {V_DOT_INPUTED:.4f}")
    print(f"{'Inputed Angular Accel (rad/s^2):':<35} {W_DOT_INPUTED:.4f}\n")

    print("OUTPUTS (Calculated Torques):")
    print("-----------------------------")
    print(f"{'Required Right Torque (Nm):':<35} {required_tau_r:.4f}")
    print(f"{'Required Left Torque (Nm):':<35} {required_tau_l:.4f}\n")

    print("VERIFICATION FROM SIMULATION:")
    print("-----------------------------")
    print(f"The simulation applied the above linear and angular accelerations, resulting in an average motion.")
    print("We now use that average motion to recalculate the theoretical torques:\n")
    
    # Get the average actual accelerations from the simulation log
    avg_v_dot_act = np.mean(log_v_dot_act)
    avg_w_dot_act = np.mean(log_w_dot_act)

    # Use the inverse dynamics model to find the theoretical torques for the ACTUAL motion
    theoretical_tau_r, theoretical_tau_l = robot.calculate_inverse_dynamics(avg_v_dot_act, avg_w_dot_act)
    
    # Compare the theoretical torques to the ones we originally calculated and used
    tau_r_error = (abs(theoretical_tau_r - required_tau_r) / (abs(required_tau_r) if required_tau_r != 0 else 1)) * 100
    tau_l_error = (abs(theoretical_tau_l - required_tau_l) / (abs(required_tau_l) if required_tau_l != 0 else 1)) * 100

    print(f"{'Parameter':<25} {'Initial Output':<15} {'Theoretical':<15} {'Error (%)':<10}")
    print("-"*30)
    print(f"{'Right Torque (Nm)':<25} {required_tau_r:<15.4f} {theoretical_tau_r:<15.4f} {tau_r_error:<10.4f}")
    print(f"{'Left Torque (Nm)':<25} {required_tau_l:<15.4f} {theoretical_tau_l:<15.4f} {tau_l_error:<10.4f}")


if __name__ == "__main__":
    main()