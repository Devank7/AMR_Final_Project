import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import math
import numpy as np
import csv

SCREEN_WIDTH, SCREEN_HEIGHT = 1000, 800
WHITE, BLACK, RED, GREEN, BLUE = (255, 255, 255), (0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255)

DT = 0.01  
SIMULATION_DURATION = 20  

PX_PER_METER = 100

M_REAL = 5.0  
W_REAL = 0.5  
IC_REAL = M_REAL * (W_REAL**2 + W_REAL**2) / 12  # kg*m^2

# Controller'S Assumed Parameters (The Uncertainty)
M_ASSUMED = 4.0  # (The controller's model is wrong)
W_ASSUMED = 0.5  # meters
IC_ASSUMED = M_ASSUMED * (W_ASSUMED**2 + W_ASSUMED**2) / 12  

R = 0.05  
L = 0.6   


ROBOT_WIDTH_PX = int(W_ASSUMED * PX_PER_METER)
WHEEL_RADIUS_PX = int(R * PX_PER_METER)

class Robot:
    """
    Represents the "Real" vehicle model.
    Its physics are governed by the TRUE parameters (M_REAL, IC_REAL).
    """
    def __init__(self, x, y, theta=0):
        # State variables
        self.x, self.y, self.theta = x, y, theta  
        self.v, self.w = 0.0, 0.0  

        self.m = M_REAL
        self.Ic = IC_REAL
        self.r = R
        self.L = L

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

    def update_physics(self, tau_r, tau_l, d_v, d_w, dt):
        """
        FORWARD DYNAMICS: This is the "real" physics.
        It uses REAL parameters and includes DISTURBANCES.
        """
        #  Forward Dynamics Model
        v_dot_actual = (tau_r + tau_l) / (self.m * self.r) + d_v
        w_dot_actual = (self.L * (tau_r - tau_l)) / (2 * self.Ic * self.r) + d_w
        
        self.v += v_dot_actual * dt
        self.w += w_dot_actual * dt

        # Integrate velocities to get new pose
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

class SlidingModeController:
    """
    Implements the SMC controller and Feedback Linearization.
    It uses the ASSUMED parameters (M_ASSUMED, IC_ASSUMED).
    """
    def __init__(self, K_v, K_w):
        self.K_v = K_v  # Robust gain for linear velocity
        self.K_w = K_w  # Robust gain for angular velocity
        
        self.m = M_ASSUMED
        self.Ic = IC_ASSUMED
        self.r = R
        self.L = L

    def calculate_control(self, v, w, v_d, v_dot_d, w_d, w_dot_d):
        """Calculates the control inputs u_v and u_w."""
        
        # Linear Velocity Control
        e_v = v - v_d
        s_v = e_v  # Sliding surface for 1st-order system
        
        u_eq_v = v_dot_d  # Equivalent control
        u_rob_v = -self.K_v * np.sign(s_v) # Robust control
        u_v = u_eq_v + u_rob_v
        
        # Angular Velocity Control 
        e_w = w - w_d
        s_w = e_w # Sliding surface for 1st-order system
        
        u_eq_w = w_dot_d  # Equivalent control
        u_rob_w = -self.K_w * np.sign(s_w) # Robust control
        u_w = u_eq_w + u_rob_w
        
        return u_v, u_w

    def inverse_dynamics(self, u_v, u_w):
        """
        Feedback Linearization (Inverse Dynamics).
        Calculates required torques using ASSUMED parameters.
        """
        tau_r = (self.m * self.r * u_v / 2) + (self.Ic * self.r * u_w / self.L)
        tau_l = (self.m * self.r * u_v / 2) - (self.Ic * self.r * u_w / self.L)
        return tau_r, tau_l

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Sliding Mode Control Sim (with Uncertainty & Disturbances)")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 28)

    # Setup 
    robot = Robot(SCREEN_WIDTH // 2, SCREEN_HEIGHT - 100, theta=math.pi/4) 
    
    # Define disturbance bounds
    D_V_MAX = 2.0  # Max expected disturbance (m/s^2)
    D_W_MAX = 1.0  
    
    # Set controller gain K > max disturbance
    controller = SlidingModeController(K_v=D_V_MAX * 1.2, K_w=D_W_MAX * 1.2)
    
    phase_portrait_data = [] 

    running = True
    sim_time = 0
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        if sim_time >= SIMULATION_DURATION:
            running = False
            continue

        if sim_time < 5.0:
            v_d = 0.2 * sim_time  # Desired linear velocity
            v_dot_d = 0.2         # Desired linear acceleration
        else:
            v_d = 1.0
            v_dot_d = 0.0
        
        w_d = 0.0     # Desired angular velocity
        w_dot_d = 0.0 # Desired angular acceleration

        d_v = D_V_MAX * math.sin(sim_time * 1.5)
        d_w = D_W_MAX * math.cos(sim_time * 1.0)

        u_v, u_w = controller.calculate_control(
            robot.v, robot.w,
            v_d, v_dot_d,
            w_d, w_dot_d
        )

        # Controller Calculates Torques (using F.L.)
        tau_r, tau_l = controller.inverse_dynamics(u_v, u_w)

        v_dot_actual, w_dot_actual = robot.update_physics(
            tau_r, tau_l, 
            d_v, d_w, 
            DT
        )

        # Log Data for Phase Portrait
        e_v = robot.v - v_d
        e_dot_v = v_dot_actual - v_dot_d
        e_w = robot.w - w_d
        e_dot_w = w_dot_actual - w_dot_d
        phase_portrait_data.append([sim_time, e_v, e_dot_v, e_w, e_dot_w])

        screen.fill(WHITE)
        robot.draw(screen)

        # Display Info
        info = [
            f"Time: {sim_time:.2f}s / {SIMULATION_DURATION}s",
            f"STATE: v={robot.v:.2f} m/s | w={robot.w:.2f} rad/s",
            f"DESIRED: v_d={v_d:.2f} m/s | w_d={w_d:.2f} rad/s",
            f"ERROR: e_v={e_v:.2f} | e_w={e_w:.2f}",
            f"DISTURBANCE: d_v={d_v:.2f} | d_w={d_w:.2f}",
            f"TORQUE: R={tau_r:.2f} Nm | L={tau_l:.2f} Nm",
        ]
        for i, text in enumerate(info):
            screen.blit(font.render(text, True, BLACK), (10, 10 + i * 30))

        pygame.display.flip()
        sim_time += DT
        clock.tick(1 / DT)
        
    pygame.quit()
    
    # Saving Data for the part 
    csv_filename = "phase_portrait_data.csv"
    with open(csv_filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["time", "e_v", "e_dot_v", "e_w", "e_dot_w"])
        writer.writerows(phase_portrait_data)
        
    print(f"\nSimulation complete.")
    print(f"Data for phase portraits saved to: {csv_filename}")
    print("We can now plot 'e_v' (x-axis) vs. 'e_dot_v' (y-axis) from this file.")


if __name__ == "__main__":
    main()