import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import math
import numpy as np

SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
WHITE, BLACK, RED, GREEN, BLUE, GRAY = (255, 255, 255), (0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255), (200, 200, 200)

DT = 0.03  
PX_PER_METER = 50  

V_MAX = 2.0         
W_MAX = 2.0         
D_SAFE_M = 2.5      # Safety radius in meters
GAMMA = 2.0         # CBF Gain

ROBOT_WIDTH_M = 0.8       
WHEEL_RADIUS_M = 0.15     
ROBOT_WIDTH_PX = int(ROBOT_WIDTH_M * PX_PER_METER)
WHEEL_RADIUS_PX = int(WHEEL_RADIUS_M * PX_PER_METER)

class Robot:
    def __init__(self, x, y, theta=0):
        self.x, self.y, self.theta = x, y, theta
        self.v, self.w = 0.0, 0.0
        
        # Visualization 
        surface_width = ROBOT_WIDTH_PX + (WHEEL_RADIUS_PX * 2)
        surface_height = ROBOT_WIDTH_PX
        self.robot_surf = pygame.Surface((surface_width, surface_height), pygame.SRCALPHA)
        self._draw_static_shape()
        self.trace = []

    def _draw_static_shape(self):
        body_rect = pygame.Rect(WHEEL_RADIUS_PX, 0, ROBOT_WIDTH_PX, ROBOT_WIDTH_PX)
        pygame.draw.rect(self.robot_surf, BLUE, body_rect)
        pygame.draw.circle(self.robot_surf, RED, (WHEEL_RADIUS_PX, 0), WHEEL_RADIUS_PX)
        pygame.draw.circle(self.robot_surf, RED, (WHEEL_RADIUS_PX, ROBOT_WIDTH_PX), WHEEL_RADIUS_PX)
        center_x = self.robot_surf.get_width() / 2
        center_y = self.robot_surf.get_height() / 2
        end_x = center_x + (ROBOT_WIDTH_PX / 2)
        pygame.draw.line(self.robot_surf, GREEN, (center_x, center_y), (end_x, center_y), 3)

    def update(self, v, w, dt):
        self.v = v
        self.w = w
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt
        
        self.trace.append((int(self.x * PX_PER_METER), int(SCREEN_HEIGHT - self.y * PX_PER_METER)))
        if len(self.trace) > 500: self.trace.pop(0)

    def draw(self, screen):
        rotated_surf = pygame.transform.rotate(self.robot_surf, math.degrees(self.theta))
        screen_x = int(self.x * PX_PER_METER)
        screen_y = int(SCREEN_HEIGHT - self.y * PX_PER_METER)
        new_rect = rotated_surf.get_rect(center=(screen_x, screen_y))
        screen.blit(rotated_surf, new_rect.topleft)
        if len(self.trace) > 1:
            pygame.draw.lines(screen, BLUE, False, self.trace, 2)

def get_nominal_control(robot, goal_x, goal_y, ped_x, ped_y):
    """
    Performance Layer (Updated):
    Uses 'Potential Fields' to generate a nominal control that steers 
    AORUND obstacles instead of just straight into them.
    """
    # Attractive Force (To Goal)
    dx_goal = goal_x - robot.x
    dy_goal = goal_y - robot.y
    dist_goal = math.hypot(dx_goal, dy_goal)
    
    # Normalize goal vector
    if dist_goal > 0:
        fx_goal = dx_goal / dist_goal
        fy_goal = dy_goal / dist_goal
    else:
        fx_goal, fy_goal = 0, 0
    
    # Repulsive Force
    dx_ped = robot.x - ped_x  # Vector pointing AWAY from ped
    dy_ped = robot.y - ped_y
    dist_ped = math.hypot(dx_ped, dy_ped)
    
    fx_rep, fy_rep = 0, 0
    
    # Influence radius: The robot starts caring about the obstacle 
    influence_radius = D_SAFE_M + 2.0
    
    if dist_ped < influence_radius:
        # Strength of repulsion increases as we get closer
        strength = 3.0 * (influence_radius - dist_ped) / dist_ped
        fx_rep = (dx_ped / dist_ped) * strength
        fy_rep = (dy_ped / dist_ped) * strength
    
    # Combine Forces
    fx_total = fx_goal + fx_rep
    fy_total = fy_goal + fy_rep
    
    target_angle = math.atan2(fy_total, fx_total)
    
    angle_error = target_angle - robot.theta
    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
    
    # P-Controller for angular velocity
    w_des = 4.0 * angle_error
    
    # Linear velocity depends on goal distance (slowing down if turning sharply)
    v_des = 1.0 * min(dist_goal, 2.0) 
    if abs(angle_error) > 0.5: # if we need to turn a lot, slow down
        v_des *= 0.5
        
    # limits
    v_des = max(0, min(v_des, V_MAX)) 
    w_des = max(-W_MAX, min(w_des, W_MAX))
    
    return v_des, w_des

def safety_filter_cbf(robot, v_des, w_des, ped_x, ped_y):
    """Safety Layer (ASIF): Unchanged."""
    dx = robot.x - ped_x
    dy = robot.y - ped_y
    dist = math.hypot(dx, dy)
    
    h = dist - D_SAFE_M
    
    if h <= 0: return 0.0, w_des
    
    Lg_h_v = ((dx * math.cos(robot.theta)) + (dy * math.sin(robot.theta))) / dist
    
    barrier_constraint = (Lg_h_v * v_des) + (GAMMA * h)
    
    if barrier_constraint >= 0:
        return v_des, w_des
    else:
        v_limit = (-GAMMA * h) / Lg_h_v
        v_safe = max(0, min(v_des, v_limit))
        return v_safe, w_des

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("CBF Simulation with Potential Field Navigation")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 30)

    # Setup
    start_x, start_y = 2.0, 2.0
    goal_x, goal_y = 14.0, 10.0
    ped_x, ped_y = 8.0, 6.0 
    
    robot = Robot(start_x, start_y, theta=0.6) 
    
    running = True
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Nominal Control 
        v_des, w_des = get_nominal_control(robot, goal_x, goal_y, ped_x, ped_y)
        
        # Safety Filter (The Mathematical Guarantee)
        v_safe, w_safe = safety_filter_cbf(robot, v_des, w_des, ped_x, ped_y)
        
        robot.update(v_safe, w_safe, DT)
        
        screen.fill(WHITE)
        
        def to_screen(x, y):
            return int(x * PX_PER_METER), int(SCREEN_HEIGHT - y * PX_PER_METER)

        gx, gy = to_screen(goal_x, goal_y)
        pygame.draw.circle(screen, GREEN, (gx, gy), 15)
        screen.blit(font.render("GOAL", True, BLACK), (gx-20, gy-30))
        
        # Pedestrian
        px, py = to_screen(ped_x, ped_y)
        pygame.draw.circle(screen, RED, (px, py), 15)
        screen.blit(font.render("PEDESTRIAN", True, BLACK), (px-40, py-30))
        
        # Draw Safety Boundary
        safe_radius_px = int(D_SAFE_M * PX_PER_METER)
        pygame.draw.circle(screen, RED, (px, py), safe_radius_px, 2)
        
        influence_radius_px = int((D_SAFE_M + 2.0) * PX_PER_METER)
        pygame.draw.circle(screen, GRAY, (px, py), influence_radius_px, 1)

        robot.draw(screen)
        
        # Info
        dist = math.hypot(robot.x - ped_x, robot.y - ped_y)
        h_val = dist - D_SAFE_M
        
        safety_active = v_safe < (v_des - 0.01)
        status_color = RED if safety_active else GREEN
        status_text = "SAFETY ACTIVE (Braking)" if safety_active else "NORMAL OPERATION"
        
        texts = [
            f"Mode: {status_text}",
            f"Distance to Ped: {dist:.2f} m",
            f"Barrier h(x): {h_val:.2f}",
            f"Nominal V: {v_des:.2f} m/s",
            f"Safe V:    {v_safe:.2f} m/s"
        ]
        
        for i, t in enumerate(texts):
            col = status_color if i == 0 else BLACK
            screen.blit(font.render(t, True, col), (10, 10 + i * 25))

        pygame.display.flip()
        clock.tick(int(1/DT))

    pygame.quit()

if __name__ == "__main__":
    main()