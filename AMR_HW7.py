import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import math
import numpy as np

SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
WHITE, BLACK, RED, GREEN, BLUE, GRAY = (255, 255, 255), (0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255), (200, 200, 200)

DT = 0.05
PX_PER_METER = 40

# Potential Field Parameters
K_ATT = 2.0  # Attractive gain
K_REP = 100.0 # Repulsive gain
REP_RANGE = 2.5 
MAX_FORCE = 5.0 

ROBOT_WIDTH_M = 0.6
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
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        
        self.trace.append((int(self.x * PX_PER_METER), int(SCREEN_HEIGHT - self.y * PX_PER_METER)))
        if len(self.trace) > 1000: self.trace.pop(0)

    def draw(self, screen):
        rotated_surf = pygame.transform.rotate(self.robot_surf, math.degrees(self.theta))
        screen_x = int(self.x * PX_PER_METER)
        screen_y = int(SCREEN_HEIGHT - self.y * PX_PER_METER)
        new_rect = rotated_surf.get_rect(center=(screen_x, screen_y))
        screen.blit(rotated_surf, new_rect.topleft)
        if len(self.trace) > 1:
            pygame.draw.lines(screen, BLUE, False, self.trace, 2)

class Obstacle:
    def __init__(self, x, y, radius=0.5):
        self.x = x
        self.y = y
        self.radius = radius 
    
    def get_repulsive_force(self, rx, ry):
        dx = rx - self.x
        dy = ry - self.y
        dist = math.hypot(dx, dy)
        
        dist_surface = dist - self.radius
        
        if dist_surface <= 0: 
             # Pushing away hard
             angle = math.atan2(dy, dx)
             return MAX_FORCE * math.cos(angle), MAX_FORCE * math.sin(angle)

        if dist_surface < REP_RANGE:
            magnitude = K_REP * (1.0/dist_surface - 1.0/REP_RANGE) * (1.0/(dist_surface**2))
            
            # Direction away from obstacle
            fx = (dx / dist) * magnitude
            fy = (dy / dist) * magnitude
            return fx, fy
        return 0.0, 0.0

    def draw(self, screen):
        sx = int(self.x * PX_PER_METER)
        sy = int(SCREEN_HEIGHT - self.y * PX_PER_METER)
        rad_px = int(self.radius * PX_PER_METER)
        pygame.draw.circle(screen, RED, (sx, sy), rad_px)
        range_px = int((self.radius + REP_RANGE) * PX_PER_METER)
        pygame.draw.circle(screen, GRAY, (sx, sy), range_px, 1)

class UShapeObstacle:
    """Represents the U-shaped trap from Slide 27"""
    def __init__(self, x, y, width=2.0, height=2.0, thickness=0.5):
        self.x, self.y = x, y 
        self.parts = []
        
        # Left wall
        self.parts.append(Obstacle(x - width/2, y + height/2, radius=thickness))
        self.parts.append(Obstacle(x - width/2, y, radius=thickness))
        # Right wall
        self.parts.append(Obstacle(x + width/2, y + height/2, radius=thickness))
        self.parts.append(Obstacle(x + width/2, y, radius=thickness))
        # Bottom wall
        self.parts.append(Obstacle(x, y - height/2 + thickness, radius=thickness))
        
    def get_repulsive_force(self, rx, ry):
        fx_tot, fy_tot = 0, 0
        for part in self.parts:
            fx, fy = part.get_repulsive_force(rx, ry)
            fx_tot += fx
            fy_tot += fy
        return fx_tot, fy_tot

    def draw(self, screen):
        for part in self.parts:
            part.draw(screen)

def potential_field_control(robot, goal_x, goal_y, obstacles):
    # Attractive Force (To Goal)
    dx = goal_x - robot.x
    dy = goal_y - robot.y
    dist = math.hypot(dx, dy)
    
    fx_att = K_ATT * dx
    fy_att = K_ATT * dy
    
    # Repulsive Force (From Obstacles)
    fx_rep, fy_rep = 0, 0
    for obs in obstacles:
        fx, fy = obs.get_repulsive_force(robot.x, robot.y)
        fx_rep += fx
        fy_rep += fy
        
    # Total Force
    fx_total = fx_att + fx_rep
    fy_total = fy_att + fy_rep
    
   
    force_mag = math.hypot(fx_total, fy_total)
    if force_mag > MAX_FORCE:
        scale = MAX_FORCE / force_mag
        fx_total *= scale
        fy_total *= scale
        
    # Convert Force to Robot Control (v, w)
    target_heading = math.atan2(fy_total, fx_total)
    
    heading_err = target_heading - robot.theta
    heading_err = (heading_err + math.pi) % (2 * math.pi) - math.pi
    
    w = 4.0 * heading_err
    
    # If we are pointing in right direction, go fast. If not, slow down.
    alignment = max(0, math.cos(heading_err))
    v = 0.5 * force_mag * alignment
    
    # If very close to goal, stop
    if dist < 0.2:
        v, w = 0, 0
        
    return v, w, fx_total, fy_total

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Potential Fields (Success vs Failure)")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 30)

    # Scenario 1: Success
    robot1 = Robot(2.0, 7.0, theta=0)
    goal1 = (18.0, 7.0)
    obstacles1 = [Obstacle(10.0, 7.0, radius=1.0)] # Obstacle directly in path
    
    # Scenario 2: Failure
    robot2 = Robot(2.0, 3.0, theta=0)
    goal2 = (18.0, 3.0)

    trap_x = 10.0
    trap_y = 3.0
    obstacles2 = [
        # Top wall
        Obstacle(trap_x, trap_y + 1.5, radius=0.6), 
        Obstacle(trap_x - 1.0, trap_y + 1.5, radius=0.6),
        # Bottom wall
        Obstacle(trap_x, trap_y - 1.5, radius=0.6),
        Obstacle(trap_x - 1.0, trap_y - 1.5, radius=0.6),
        # Back wall 
        Obstacle(trap_x + 0.5, trap_y, radius=0.8),
        Obstacle(trap_x + 0.5, trap_y + 1.0, radius=0.6),
        Obstacle(trap_x + 0.5, trap_y - 1.0, radius=0.6),
    ]

    current_scenario = 1
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    current_scenario = 1
                    robot1 = Robot(2.0, 7.0, theta=0) 
                elif event.key == pygame.K_2:
                    current_scenario = 2
                    robot2 = Robot(2.0, 3.0, theta=0)

        screen.fill(WHITE)
        
        if current_scenario == 1:
            robot = robot1
            goal = goal1
            obstacles = obstacles1
            title = "Scenario 1: SUCCESS (Simple Obstacle)"
        else:
            robot = robot2
            goal = goal2
            obstacles = obstacles2
            title = "Scenario 2: FAILURE (Local Minimum Trap)"
            
        # Control logic
        v, w, fx, fy = potential_field_control(robot, goal[0], goal[1], obstacles)
        robot.update(v, w, DT)
        
        
        gx = int(goal[0] * PX_PER_METER)
        gy = int(SCREEN_HEIGHT - goal[1] * PX_PER_METER)
        pygame.draw.circle(screen, GREEN, (gx, gy), 15)
        screen.blit(font.render("GOAL", True, BLACK), (gx-20, gy-30))
        
        for obs in obstacles:
            obs.draw(screen)
            
        robot.draw(screen)
        
        # Visualizing the pull
        start_pos = (int(robot.x * PX_PER_METER), int(SCREEN_HEIGHT - robot.y * PX_PER_METER))
        end_pos = (start_pos[0] + int(fx * 20), start_pos[1] - int(fy * 20))
        pygame.draw.line(screen, BLACK, start_pos, end_pos, 3)
        
        screen.blit(font.render(title, True, BLACK), (20, 20))
        screen.blit(font.render("Success Scenario, Failure Scenario", True, BLUE), (20, 50))
        
        if current_scenario == 2 and robot.x > 8.0 and robot.x < 11.0:
             screen.blit(font.render("LOCAL MINIMUM DETECTED! Forces balanced.", True, RED), (20, 80))

        pygame.display.flip()
        clock.tick(int(1/DT))

    pygame.quit()

if __name__ == "__main__":
    main()