import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import math
import random
import numpy as np

SCREEN_WIDTH, SCREEN_HEIGHT = 1000, 700

BG_TOP = (20, 24, 35)       
BG_BOTTOM = (10, 12, 18)   
GRID_COLOR = (40, 50, 60)
TEXT_COLOR = (200, 255, 255)

ROBOT_COLOR = (0, 190, 255)       
ROBOT_GLOW = (0, 100, 200)
ROBOT_TRACE = (0, 190, 255)

WALL_COLOR = (45, 55, 70)         
WALL_BORDER = (80, 100, 120)

PEDESTRIAN_COLOR = (255, 50, 80)  
GOAL_COLOR = (50, 255, 100)       
OBSTACLE_COLOR = (200, 200, 200)  

PREDICTED_PATH_COLOR = (0, 255, 255, 100) 

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 50, 50)
GREEN = (50, 255, 50)

DT = 0.05
PX_PER_METER = 40

K_ATT = 2.0   
K_REP = 120.0 
REP_RANGE = 2.5 
MAX_FORCE = 8.0 

ROBOT_WIDTH_M = 0.6
WHEEL_RADIUS_M = 0.15

ROBOT_WIDTH_PX = int(ROBOT_WIDTH_M * PX_PER_METER)
WHEEL_RADIUS_PX = int(WHEEL_RADIUS_M * PX_PER_METER)


def draw_shadow(screen, surface, x, y, offset=5):
    mask = pygame.mask.from_surface(surface)
    shadow_surf = mask.to_surface(setcolor=(0, 0, 0, 80), unsetcolor=(0,0,0,0))
    screen.blit(shadow_surf, (x + offset, y + offset))

def create_glow_circle(radius, color):
    surf = pygame.Surface((radius*2, radius*2), pygame.SRCALPHA)
    for r in range(radius, 0, -2):
        alpha = int(255 * (1 - (r/radius)**2)) 
        col = color + (alpha,)
        pygame.draw.circle(surf, col, (radius, radius), r)
    return surf

def draw_gradient_rect(screen, rect, top_color, bot_color):
    h = rect.height
    grad_surf = pygame.Surface((1, h))
    for y in range(h):
        r = top_color[0] + (bot_color[0] - top_color[0]) * y // h
        g = top_color[1] + (bot_color[1] - top_color[1]) * y // h
        b = top_color[2] + (bot_color[2] - top_color[2]) * y // h
        grad_surf.set_at((0, y), (r, g, b))
    
    scaled_grad = pygame.transform.scale(grad_surf, (rect.width, h))
    screen.blit(scaled_grad, rect)
    pygame.draw.rect(screen, WALL_BORDER, rect, 2) 

class Button:
    def __init__(self, x, y, w, h, text, color=(60, 80, 100)):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.base_color = color
        self.hover_color = (min(color[0]+40, 255), min(color[1]+40, 255), min(color[2]+40, 255))
        self.font = pygame.font.Font(None, 24)

    def draw(self, screen):
        mouse_pos = pygame.mouse.get_pos()
        col = self.hover_color if self.rect.collidepoint(mouse_pos) else self.base_color
        
        shadow_rect = self.rect.copy()
        shadow_rect.move_ip(3, 3)
        pygame.draw.rect(screen, (0,0,0,100), shadow_rect, border_radius=5)
        
        pygame.draw.rect(screen, col, self.rect, border_radius=5)
        pygame.draw.rect(screen, (200, 255, 255), self.rect, 1, border_radius=5)
        
        txt_surf = self.font.render(self.text, True, TEXT_COLOR)
        txt_rect = txt_surf.get_rect(center=self.rect.center)
        screen.blit(txt_surf, txt_rect)

    def is_clicked(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                return True
        return False

class Robot:
    def __init__(self, x, y, theta=0):
        self.x, self.y, self.theta = x, y, theta
        self.v, self.w = 0.0, 0.0
        
        surface_width = ROBOT_WIDTH_PX + (WHEEL_RADIUS_PX * 2)
        surface_height = ROBOT_WIDTH_PX
        self.robot_surf = pygame.Surface((surface_width, surface_height), pygame.SRCALPHA)
        self._draw_static_shape()
        self.trace = []

    def _draw_static_shape(self):
        glow_radius = int(ROBOT_WIDTH_PX * 0.8)
        
        pygame.draw.circle(self.robot_surf, (20, 20, 20), (WHEEL_RADIUS_PX, 0), WHEEL_RADIUS_PX)
        pygame.draw.circle(self.robot_surf, (20, 20, 20), (WHEEL_RADIUS_PX, ROBOT_WIDTH_PX), WHEEL_RADIUS_PX)
        
        center_x = self.robot_surf.get_width() / 2
        center_y = self.robot_surf.get_height() / 2
        
        body_rect = pygame.Rect(WHEEL_RADIUS_PX, 0, ROBOT_WIDTH_PX, ROBOT_WIDTH_PX)
        pygame.draw.rect(self.robot_surf, ROBOT_COLOR, body_rect, border_radius=5)
        pygame.draw.rect(self.robot_surf, (200, 255, 255), body_rect, 2, border_radius=5) # Highlight

        end_x = center_x + (ROBOT_WIDTH_PX / 2) - 2
        pygame.draw.line(self.robot_surf, (255, 255, 255), (center_x, center_y), (end_x, center_y), 3)

    def update_kinematics(self, v_des, w_des, dt):
        alpha = 0.2
        self.v += (v_des - self.v) * alpha
        self.w += (w_des - self.w) * alpha
        
        self.theta += self.w * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        
        self.trace.append((int(self.x * PX_PER_METER), int(SCREEN_HEIGHT - self.y * PX_PER_METER)))
        if len(self.trace) > 1000: self.trace.pop(0)

    def draw(self, screen):
        rotated_surf = pygame.transform.rotate(self.robot_surf, math.degrees(self.theta))
        screen_x = int(self.x * PX_PER_METER)
        screen_y = int(SCREEN_HEIGHT - self.y * PX_PER_METER)
        new_rect = rotated_surf.get_rect(center=(screen_x, screen_y))
        
        draw_shadow(screen, rotated_surf, new_rect.x, new_rect.y)
        
        screen.blit(rotated_surf, new_rect.topleft)
        
        if len(self.trace) > 1:
            pygame.draw.lines(screen, ROBOT_TRACE, False, self.trace, 2)


class Obstacle:
    def get_repulsion(self, rx, ry): return 0,0
    def draw(self, screen, robot): pass
    def get_distance(self, rx, ry): return 999
    def check_collision(self, x, y, radius): return False

class WallObstacle(Obstacle):
    def __init__(self, x, y, w, h):
        self.rect = pygame.Rect(x * PX_PER_METER, SCREEN_HEIGHT - (y * PX_PER_METER) - (h * PX_PER_METER), 
                                w * PX_PER_METER, h * PX_PER_METER)
        self.x_m, self.y_m = x, y 
        self.w_m, self.h_m = w, h

    def get_repulsion(self, rx, ry):
        left = self.x_m
        right = self.x_m + self.w_m
        bottom = self.y_m
        top = self.y_m + self.h_m
        
        closest_x = max(left, min(rx, right))
        closest_y = max(bottom, min(ry, top))
        
        dx = rx - closest_x
        dy = ry - closest_y
        dist = math.hypot(dx, dy)
        
        if dist < 0.01: 
             cx = left + self.w_m/2
             cy = bottom + self.h_m/2
             angle = math.atan2(ry - cy, rx - cx)
             return MAX_FORCE * math.cos(angle), MAX_FORCE * math.sin(angle)

        if dist < REP_RANGE:
            mag = K_REP * (1.0/dist - 1.0/REP_RANGE) * (1.0/(dist**2))
            return (dx/dist) * mag, (dy/dist) * mag
        return 0.0, 0.0
    
    def check_collision(self, rx, ry, radius):
        closest_x = max(self.x_m, min(rx, self.x_m + self.w_m))
        closest_y = max(self.y_m, min(ry, self.y_m + self.h_m))
        
        dx = rx - closest_x
        dy = ry - closest_y
        return (dx*dx + dy*dy) < (radius*radius)

    def draw(self, screen, robot):
        shadow_rect = self.rect.copy()
        shadow_rect.move_ip(5, 5)
        pygame.draw.rect(screen, (0,0,0,100), shadow_rect, border_radius=3)
        draw_gradient_rect(screen, self.rect, (60, 70, 90), WALL_COLOR)

class Pedestrian(Obstacle):
    def __init__(self, x, y, radius=0.4, label="Pedestrian"):
        self.x, self.y = x, y
        self.radius = radius
        self.label = label
        self.font = pygame.font.Font(None, 20)
        self.vx = random.uniform(-0.5, 0.5)
        self.vy = random.uniform(-0.5, 0.5)

    def update(self, dt, walls):
        # Propose new position
        next_x = self.x + self.vx * dt
        next_y = self.y + self.vy * dt
        
        hit_boundary = False
        if next_x < 1.0 or next_x > (SCREEN_WIDTH/PX_PER_METER)-1.0: 
            self.vx *= -1
            hit_boundary = True
        if next_y < 1.0 or next_y > (SCREEN_HEIGHT/PX_PER_METER)-1.0: 
            self.vy *= -1
            hit_boundary = True
            
        hit_wall = False
        if not hit_boundary:
            for w in walls:
                if w.check_collision(next_x, next_y, self.radius):
                    hit_wall = True
                    break
        
        if hit_wall:
            # bounce
            self.vx *= -1
            self.vy *= -1
            # Move slightly to unstick
            self.x += self.vx * dt
            self.y += self.vy * dt
        elif not hit_boundary:
            self.x = next_x
            self.y = next_y

    def get_repulsion(self, rx, ry):
        dx = rx - self.x
        dy = ry - self.y
        dist = math.hypot(dx, dy)
        dist_surf = dist - self.radius
        
        if dist_surf <= 0: 
             a = math.atan2(dy, dx)
             return MAX_FORCE * math.cos(a), MAX_FORCE * math.sin(a)

        if dist_surf < REP_RANGE:
            mag = K_REP * (1.0/dist_surf - 1.0/REP_RANGE) * (1.0/(dist_surf**2))
            return (dx/dist) * mag, (dy/dist) * mag
        return 0.0, 0.0
    
    def get_distance(self, rx, ry):
        return math.hypot(self.x - rx, self.y - ry) - self.radius

    def draw(self, screen, robot):
        sx, sy = int(self.x * PX_PER_METER), int(SCREEN_HEIGHT - self.y * PX_PER_METER)
        
        # Dynamic boundary
        if robot:
            dist = math.hypot(self.x - robot.x, self.y - robot.y)
            if dist < (REP_RANGE + self.radius + 0.5):
                radius_px = int((self.radius + REP_RANGE) * PX_PER_METER)
                pygame.draw.circle(screen, (200, 50, 50), (sx, sy), radius_px, 1)

        radius_px = int(self.radius * PX_PER_METER)
        glow = create_glow_circle(radius_px + 5, PEDESTRIAN_COLOR)
        screen.blit(glow, (sx - radius_px - 5, sy - radius_px - 5))
        pygame.draw.circle(screen, (255, 200, 200), (sx, sy), int(radius_px*0.5))
        
        text_surf = self.font.render(self.label, True, TEXT_COLOR)
        screen.blit(text_surf, (sx + 15, sy - 15))

class UShapeTrap:
    def __init__(self, x, y):
        self.x, self.y = x, y
        w, h = 3.0, 3.0
        th = 0.3
        self.walls = []
        
        self.walls.append(WallObstacle(x + w/2 - th, y - h/2, th, h)) 
        self.walls.append(WallObstacle(x - w/2, y + h/2 - th, w, th))
        self.walls.append(WallObstacle(x - w/2, y - h/2, w, th))
        
        self.width, self.height = w, h

    def get_repulsion(self, rx, ry):
        fx_tot, fy_tot = 0, 0
        for w in self.walls:
            fx, fy = w.get_repulsion(rx, ry)
            fx_tot += fx
            fy_tot += fy
        return fx_tot, fy_tot

    def draw(self, screen, robot):
        for w in self.walls:
            w.draw(screen, robot)

    def is_inside(self, rx, ry):
        return (self.x - self.width/2 - 1.5 < rx < self.x + self.width/2) and \
               (self.y - self.height/2 < ry < self.y + self.height/2)
               
    def get_distance(self, rx, ry):
        return math.hypot(self.x - rx, self.y - ry) - 1.5

# Shapes
class CircleObstacle(Obstacle):
    def __init__(self, x, y, radius=0.6):
        self.x, self.y, self.radius = x, y, radius
    
    def get_repulsion(self, rx, ry):
        dx = rx - self.x
        dy = ry - self.y
        dist = math.hypot(dx, dy)
        dist_surf = dist - self.radius
        if dist_surf <= 0:
             a = math.atan2(dy, dx)
             return MAX_FORCE * math.cos(a), MAX_FORCE * math.sin(a)
        if dist_surf < REP_RANGE:
            mag = K_REP * (1.0/dist_surf - 1.0/REP_RANGE) * (1.0/(dist_surf**2))
            return (dx/dist) * mag, (dy/dist) * mag
        return 0.0, 0.0

    def draw(self, screen, robot):
        sx, sy = int(self.x * PX_PER_METER), int(SCREEN_HEIGHT - self.y * PX_PER_METER)
        radius_px = int(self.radius * PX_PER_METER)
        
        
        pygame.draw.circle(screen, (0,0,0,100), (sx+5, sy+5), radius_px)
        glow = create_glow_circle(radius_px + 2, OBSTACLE_COLOR)
        screen.blit(glow, (sx - radius_px - 2, sy - radius_px - 2))
        pygame.draw.circle(screen, (150, 150, 150), (sx, sy), int(radius_px*0.8))

class SquareObstacle(Obstacle):
    def __init__(self, x, y, side=1.2):
        self.x, self.y, self.side = x, y, side
        self.radius = side / 2 
    
    def get_repulsion(self, rx, ry):
        dx = rx - self.x
        dy = ry - self.y
        dist = math.hypot(dx, dy)
        dist_surf = dist - self.radius
        if dist_surf <= 0:
             a = math.atan2(dy, dx)
             return MAX_FORCE * math.cos(a), MAX_FORCE * math.sin(a)
        if dist_surf < REP_RANGE:
            mag = K_REP * (1.0/dist_surf - 1.0/REP_RANGE) * (1.0/(dist_surf**2))
            return (dx/dist) * mag, (dy/dist) * mag
        return 0.0, 0.0

    def draw(self, screen, robot):
        size_px = int(self.side * PX_PER_METER)
        sx = int((self.x - self.side/2) * PX_PER_METER)
        sy = int(SCREEN_HEIGHT - (self.y + self.side/2) * PX_PER_METER)
        
        rect = pygame.Rect(sx, sy, size_px, size_px)
        
        
        shadow_rect = rect.copy()
        shadow_rect.move_ip(5, 5)
        pygame.draw.rect(screen, (0,0,0,100), shadow_rect, border_radius=3)
        
        
        draw_gradient_rect(screen, rect, (180, 50, 180), (100, 30, 100))

class TriangleObstacle(Obstacle):
    def __init__(self, x, y, size=1.2):
        self.x, self.y, self.size = x, y, size
        self.radius = size / 2
    
    def get_repulsion(self, rx, ry):
        dx = rx - self.x
        dy = ry - self.y
        dist = math.hypot(dx, dy)
        dist_surf = dist - self.radius
        if dist_surf <= 0:
             a = math.atan2(dy, dx)
             return MAX_FORCE * math.cos(a), MAX_FORCE * math.sin(a)
        if dist_surf < REP_RANGE:
            mag = K_REP * (1.0/dist_surf - 1.0/REP_RANGE) * (1.0/(dist_surf**2))
            return (dx/dist) * mag, (dy/dist) * mag
        return 0.0, 0.0

    def draw(self, screen, robot):
        sx, sy = int(self.x * PX_PER_METER), int(SCREEN_HEIGHT - self.y * PX_PER_METER)
        r = int(self.size * PX_PER_METER)
        points = [(sx, sy - r//2), (sx - r//2, sy + r//2), (sx + r//2, sy + r//2)]
        
        shadow_points = [(p[0]+5, p[1]+5) for p in points]
        pygame.draw.polygon(screen, (0,0,0,100), shadow_points)
        
        
        pygame.draw.polygon(screen, (255, 255, 50), points)
        pygame.draw.polygon(screen, (255, 200, 0), points, 3)

# Path Prediction
def calculate_predicted_path(start_robot, goal, obstacles, walls):
    path_points = []
    vx, vy = start_robot.x, start_robot.y
    vtheta = start_robot.theta
    
    for _ in range(600): 
        screen_x = int(vx * PX_PER_METER)
        screen_y = int(SCREEN_HEIGHT - vy * PX_PER_METER)
        path_points.append((screen_x, screen_y))
        
        dx_g = goal[0] - vx
        dy_g = goal[1] - vy
        dist_g = math.hypot(dx_g, dy_g)
        
        if dist_g < 0.2: break 
        
        fx = K_ATT * dx_g
        fy = K_ATT * dy_g
        
        for obs in obstacles:
            rfx, rfy = obs.get_repulsion(vx, vy)
            fx += rfx
            fy += rfy
        for w in walls:
            rfx, rfy = w.get_repulsion(vx, vy)
            fx += rfx
            fy += rfy
        
        f_mag = math.hypot(fx, fy)
        if f_mag > MAX_FORCE:
            fx = (fx/f_mag) * MAX_FORCE
            fy = (fy/f_mag) * MAX_FORCE
        
        target_heading = math.atan2(fy, fx)
        h_err = target_heading - vtheta
        h_err = (h_err + math.pi) % (2 * math.pi) - math.pi
        
        v_step = 0.1 
        vtheta += 2.0 * h_err * v_step 
        alignment = max(0, math.cos(h_err))
        speed = 2.0 * alignment 
        
        vx += speed * math.cos(vtheta) * v_step
        vy += speed * math.sin(vtheta) * v_step

    return path_points

# HUD
def draw_hud(screen, robot, obstacles, u_trap, walls):
    panel_w, panel_h = 240, 110
    panel_x = SCREEN_WIDTH - panel_w - 20
    panel_y = 20
    
    s = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    s.fill((10, 20, 30, 200)) 
    screen.blit(s, (panel_x, panel_y))
    pygame.draw.rect(screen, (0, 255, 255), (panel_x, panel_y, panel_w, panel_h), 1) # Border
    
    font = pygame.font.Font(None, 24)
    bold_font = pygame.font.Font(None, 26)
    
    status_text = "Stopped"
    status_color = (150, 150, 150)
    
    min_dist = 999.0
    for obs in obstacles:
        d = obs.get_distance(robot.x, robot.y)
        if d < min_dist: min_dist = d
    
    if robot.v > 0.1:
        if min_dist < REP_RANGE:
            status_text = "AVOIDING OBSTACLE"
            status_color = (255, 100, 100)
        else:
            status_text = "CRUISING"
            status_color = (50, 255, 100)
    
    if u_trap.is_inside(robot.x, robot.y) and robot.v < 0.1:
        status_text = "STUCK (TRAP)"
        status_color = (255, 50, 50)
        
    v_surf = font.render(f"Velocity: {robot.v:.2f} m/s", True, TEXT_COLOR)
    screen.blit(v_surf, (panel_x + 10, panel_y + 10))
    
    l_surf = font.render("Status: ", True, TEXT_COLOR)
    screen.blit(l_surf, (panel_x + 10, panel_y + 35))
    s_surf = bold_font.render(status_text, True, status_color)
    screen.blit(s_surf, (panel_x + 70, panel_y + 35))
    
    if min_dist < 1.5:
        warn_text = f"WARNING: Obs. {min_dist:.1f}m"
        w_surf = bold_font.render(warn_text, True, (255, 50, 50))
        screen.blit(w_surf, (panel_x + 10, panel_y + 65))
    else:
        safe_surf = font.render("Path Clear", True, (50, 255, 100))
        screen.blit(safe_surf, (panel_x + 10, panel_y + 65))


def generate_environment():
    obstacles = []
    walls = []
    
    wm = SCREEN_WIDTH / PX_PER_METER
    hm = SCREEN_HEIGHT / PX_PER_METER
    walls.append(WallObstacle(0, 0, 0.5, hm))       # Left
    walls.append(WallObstacle(wm-0.5, 0, 0.5, hm))  # Right
    walls.append(WallObstacle(0, hm-0.5, wm, 0.5))  # Top
    walls.append(WallObstacle(0, 0, wm, 0.5))       # Bottom
    
    walls.append(WallObstacle(8.0, 0.0, 0.5, 6.0))
    walls.append(WallObstacle(16.0, 10.0, 8.0, 0.5))
    
    # U-Trap
    u_x = 12.0
    u_y = 8.0
    u_trap = UShapeTrap(u_x, u_y)
    
    for _ in range(4):
        while True:
            ox = random.uniform(2, wm-2)
            oy = random.uniform(2, hm-2)
            if not u_trap.is_inside(ox, oy) and \
               not (7.0 < ox < 9.0 and oy < 7.0) and \
               not (15.0 < ox < 25.0 and 9.0 < oy < 11.0):
                choice = random.choice(['circle', 'square', 'tri'])
                if choice == 'circle': obstacles.append(CircleObstacle(ox, oy, radius=0.6))
                elif choice == 'square': obstacles.append(SquareObstacle(ox, oy, side=1.2))
                else: obstacles.append(TriangleObstacle(ox, oy, size=1.5))
                break

    # Pedestrians
    ped_count = 1
    for _ in range(2):
        while True:
            px = random.uniform(2, wm-2)
            py = random.uniform(2, hm-2)
            if not u_trap.is_inside(px, py) and \
               not (7.0 < px < 9.0 and py < 7.0) and \
               not (15.0 < px < 25.0 and 9.0 < py < 11.0):
                obstacles.append(Pedestrian(px, py, label=f"Pedestrian {ped_count}"))
                ped_count += 1
                break
            
    return obstacles, u_trap, walls

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("AMR Final Project")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 30)
    big_font = pygame.font.Font(None, 50)

    restart_btn = Button(20, 20, 100, 40, "RESTART")
    
    bg_surf = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
    draw_gradient_rect(bg_surf, pygame.Rect(0,0,SCREEN_WIDTH, SCREEN_HEIGHT), BG_TOP, BG_BOTTOM)

    for x in range(0, SCREEN_WIDTH, 50):
        alpha = 50 if x % 200 == 0 else 20
        pygame.draw.line(bg_surf, (*GRID_COLOR, alpha), (x, 0), (x, SCREEN_HEIGHT))
    for y in range(0, SCREEN_HEIGHT, 50):
        alpha = 50 if y % 200 == 0 else 20
        pygame.draw.line(bg_surf, (*GRID_COLOR, alpha), (0, y), (SCREEN_WIDTH, y))

    state = 0
    robot = None
    goal = None
    obstacles, u_trap, walls = generate_environment()
    predicted_trace = []
    failure_counter = 0

    running = True
    while running:

        screen.blit(bg_surf, (0,0))
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            if restart_btn.is_clicked(event):
                state = 0
                robot = None
                goal = None
                obstacles, u_trap, walls = generate_environment()
                predicted_trace = []
                failure_counter = 0
            
            if event.type == pygame.MOUSEBUTTONDOWN and not restart_btn.rect.collidepoint(event.pos):
                mx, my = pygame.mouse.get_pos()
                wx = mx / PX_PER_METER
                wy = (SCREEN_HEIGHT - my) / PX_PER_METER
                
                if wx > 0.5 and wx < (SCREEN_WIDTH/PX_PER_METER)-0.5 and \
                   wy > 0.5 and wy < (SCREEN_HEIGHT/PX_PER_METER)-0.5:
                    if state == 0:
                        robot = Robot(wx, wy)
                        state = 1
                    elif state == 1:
                        goal = (wx, wy)
                        state = 2
                        predicted_trace = calculate_predicted_path(robot, goal, obstacles, walls + [u_trap])

 
        if state == 2:
            all_walls = walls + u_trap.walls
            for obs in obstacles:
                if isinstance(obs, Pedestrian):
                    obs.update(DT, all_walls)

        if state == 2 and robot and goal:
            # Planner
            dx_g = goal[0] - robot.x
            dy_g = goal[1] - robot.y
            dist_g = math.hypot(dx_g, dy_g)
            
            fx = K_ATT * dx_g
            fy = K_ATT * dy_g
            
            # Repulsion
            for obs in obstacles:
                rfx, rfy = obs.get_repulsion(robot.x, robot.y)
                fx += rfx
                fy += rfy
            for w in walls:
                rfx, rfy = w.get_repulsion(robot.x, robot.y)
                fx += rfx
                fy += rfy
            rfx, rfy = u_trap.get_repulsion(robot.x, robot.y)
            fx += rfx
            fy += rfy
            
            f_mag = math.hypot(fx, fy)
            if f_mag > MAX_FORCE:
                fx = (fx/f_mag) * MAX_FORCE
                fy = (fy/f_mag) * MAX_FORCE
            
            target_heading = math.atan2(fy, fx)
            h_err = target_heading - robot.theta
            h_err = (h_err + math.pi) % (2 * math.pi) - math.pi
            
            w_des = 4.0 * h_err
            alignment = max(0, math.cos(h_err))
            v_des = 0.8 * f_mag * alignment
            
            if dist_g < 0.2: v_des, w_des = 0, 0
            
            v_des = min(v_des, 2.0)
            w_des = max(-2.0, min(w_des, 2.0))

            robot.update_kinematics(v_des, w_des, DT)
            
            # Failure Check
            if u_trap.is_inside(robot.x, robot.y):
                if abs(robot.v) < 0.05 and dist_g > 0.5:
                    failure_counter += 1
                else:
                    failure_counter = 0
                if failure_counter > 60: state = 3

        
        for w in walls: w.draw(screen, robot)
        u_trap.draw(screen, robot)
        
        # Predicted Path
        if len(predicted_trace) > 1:
            pygame.draw.lines(screen, PREDICTED_PATH_COLOR, False, predicted_trace, 2)
            
        # Goal
        if goal:
            gx, gy = int(goal[0]*PX_PER_METER), int(SCREEN_HEIGHT - goal[1]*PX_PER_METER)
            
            pulse_rad = 15 + int(3 * math.sin(pygame.time.get_ticks() * 0.01))
            glow = create_glow_circle(pulse_rad + 10, GOAL_COLOR)
            screen.blit(glow, (gx - pulse_rad - 10, gy - pulse_rad - 10))
            
            pygame.draw.circle(screen, (200, 255, 200), (gx, gy), 10)
            t_surf = font.render("GOAL", True, TEXT_COLOR)
            screen.blit(t_surf, (gx+20, gy-10))

        for obs in obstacles:
            obs.draw(screen, robot)

        if robot:
            robot.draw(screen)
            if state == 2:
                draw_hud(screen, robot, obstacles, u_trap, walls)

        
        restart_btn.draw(screen)

        if state == 0:
            txt = big_font.render("CLICK TO SET START", True, ROBOT_COLOR)
            screen.blit(txt, (SCREEN_WIDTH//2 - txt.get_width()//2, 50))
        elif state == 1:
            txt = big_font.render("CLICK TO SET GOAL", True, GOAL_COLOR)
            screen.blit(txt, (SCREEN_WIDTH//2 - txt.get_width()//2, 50))
        elif state == 3:
            s = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
            s.fill((0,0,0,180)) 
            screen.blit(s, (0,0))
            
            fail_txt = big_font.render("POTENTIAL FIELD FAILURE", True, (255, 50, 50))
            desc_txt = font.render("Local Minimum Detected (Robot Stuck in U-Trap)", True, WHITE)
            
            screen.blit(fail_txt, (SCREEN_WIDTH//2 - fail_txt.get_width()//2, 300))
            screen.blit(desc_txt, (SCREEN_WIDTH//2 - desc_txt.get_width()//2, 350))

        pygame.display.flip()
        clock.tick(int(1/DT))

    pygame.quit()

if __name__ == "__main__":
    main()