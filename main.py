# main.py
import argparse
import sys
import pygame as pg
from pygame.locals import *
from random import uniform, gauss
from boid import Boid

default_boids = 100
default_geometry = "1000x1000"

# --- 全域參數控制 ---
connection_radius = 40.0 

class Obstacle(pg.sprite.Sprite):
    def __init__(self, position):
        super().__init__()
        self.image = pg.Surface((4, 4))
        self.image.fill(pg.Color('red'))
        self.rect = self.image.get_rect(center=position)
        self.position = pg.Vector2(position)

def add_obstacle_cluster(obstacles, width, height, points=20):
    cluster_x = uniform(50, width - 50)
    cluster_y = uniform(50, height - 50)
    for _ in range(points):
        ox = gauss(cluster_x, 30)
        oy = gauss(cluster_y, 30)
        ox = max(0, min(ox, width))
        oy = max(0, min(oy, height))
        obstacles.add(Obstacle((ox, oy)))

# --- Convex Hull 演算法核心 (Monotone Chain) ---
def cross_product(o, a, b):
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)

def get_convex_hull(points):
    """計算凸包頂點列表"""
    if len(points) <= 2:
        return points

    # 1. 排序
    points = sorted(points, key=lambda p: (p.x, p.y))

    # 2. 下半部
    lower = []
    for p in points:
        while len(lower) >= 2 and cross_product(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    # 3. 上半部
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross_product(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]

def get_obstacle_clusters(obstacles, radius):
    """
    回傳: 一個包含多邊形頂點列表的列表 (List of List of Vector2)
    """
    sprites = obstacles.sprites()
    if not sprites:
        return []

    clusters = []
    visited = set()

    for sprite in sprites:
        if sprite in visited:
            continue

        # BFS 分群
        current_cluster_points = []
        visited.add(sprite)
        queue = [sprite]
        current_cluster_points.append(sprite.position)

        while queue:
            current = queue.pop(0)
            for other in sprites:
                if other not in visited:
                    if current.position.distance_to(other.position) <= radius:
                        visited.add(other)
                        queue.append(other)
                        current_cluster_points.append(other.position)

        # 計算凸包
        hull = get_convex_hull(current_cluster_points)
        clusters.append(hull)

    return clusters

def update(dt, boids, obstacles, clusters, window_size):
    global connection_radius
    for event in pg.event.get():
        if event.type == QUIT:
            pg.quit()
            sys.exit(0)
        elif event.type == KEYDOWN:
            mods = pg.key.get_mods()
            if event.key == pg.K_q:
                pg.quit()
                sys.exit(0)
            
            # --- Boids Control ---
            elif event.key == pg.K_UP:
                if mods & pg.KMOD_SHIFT: add_boids(boids, 100)
                else: add_boids(boids, 10)
            elif event.key == pg.K_DOWN:
                if mods & pg.KMOD_SHIFT: boids.remove(boids.sprites()[:100])
                else: boids.remove(boids.sprites()[:10])
            elif event.key == pg.K_1:
                for b in boids: b.max_force /= 2
                print(f"Max Force: {boids.sprites()[0].max_force}")
            elif event.key == pg.K_2:
                for b in boids: b.max_force *= 2
                print(f"Max Force: {boids.sprites()[0].max_force}")
            elif event.key == pg.K_3:
                for b in boids: b.perception *= .8
                print(f"Perception: {boids.sprites()[0].perception}")
            elif event.key == pg.K_4:
                for b in boids: b.perception *= 1.2
                print(f"Perception: {boids.sprites()[0].perception}")
            elif event.key == pg.K_5:
                for b in boids: b.crowding *= 0.8
                print(f"Crowding: {boids.sprites()[0].crowding}")
            elif event.key == pg.K_6:
                for b in boids: b.crowding *= 1.2
                print(f"Crowding: {boids.sprites()[0].crowding}")
            
            # --- Obstacle Control ---
            elif event.key == pg.K_7:
                connection_radius = max(5, connection_radius - 5)
                print(f"Cluster Visual Radius: {connection_radius}")
            elif event.key == pg.K_8:
                connection_radius += 5
                print(f"Cluster Visual Radius: {connection_radius}")
            elif event.key == pg.K_9:
                for b in boids: b.avoid_weight = max(0.1, b.avoid_weight - 0.5)
                Boid.avoid_weight = boids.sprites()[0].avoid_weight
                print(f"Obstacle Avoid Weight: {Boid.avoid_weight}")
            elif event.key == pg.K_0:
                for b in boids: b.avoid_weight += 0.5
                Boid.avoid_weight = boids.sprites()[0].avoid_weight
                print(f"Obstacle Avoid Weight: {Boid.avoid_weight}")
            elif event.key == pg.K_LEFTBRACKET: 
                sprites = obstacles.sprites()
                if len(sprites) > 0:
                    obstacles.remove(sprites[:20])
                print(f"Obstacles count: {len(obstacles)}")
            elif event.key == pg.K_RIGHTBRACKET: 
                add_obstacle_cluster(obstacles, window_size[0], window_size[1])
                print(f"Obstacles count: {len(obstacles)}")
            elif event.key == pg.K_d:
                for boid in boids:
                    boid.debug = not boid.debug
            elif event.key == pg.K_r:
                num_boids = len(boids)
                boids.empty()
                add_boids(boids, num_boids)

    for b in boids:
        b.update(dt, boids, clusters)

def draw(screen, background, boids, obstacles, clusters):
    screen.blit(background, (0, 0))

    # 畫出 Convex Hull 多邊形 (取代 BBOX)
    for polygon in clusters:
        if len(polygon) >= 2:
            # 青色線條, 封閉多邊形, 線寬 2
            pg.draw.lines(screen, (0, 255, 255), True, polygon, 2)

    obstacles.draw(screen)
    boids.draw(screen)
    pg.display.flip()

def main(args):
    pg.init()
    pg.event.set_allowed([pg.QUIT, pg.KEYDOWN, pg.KEYUP])
    fps = 60.0
    fpsClock = pg.time.Clock()
    pg.display.set_caption("BOIDS - Convex Hull Avoidance")
    
    window_width, window_height = [int(x) for x in args.geometry.split("x")]
    flags = DOUBLEBUF
    screen = pg.display.set_mode((window_width, window_height), flags)
    screen.set_alpha(None)
    background = pg.Surface(screen.get_size()).convert()
    background.fill(pg.Color('black'))

    boids = pg.sprite.RenderUpdates()
    add_boids(boids, args.num_boids)

    obstacles = pg.sprite.RenderUpdates()
    for _ in range(5):
        add_obstacle_cluster(obstacles, window_width, window_height)

    dt = 1/fps

    while True:
        # 1. 計算 Convex Hull
        clusters = get_obstacle_clusters(obstacles, connection_radius)
        # 2. Update
        update(dt, boids, obstacles, clusters, (window_width, window_height))
        # 3. Draw
        draw(screen, background, boids, obstacles, clusters)
        
        dt = fpsClock.tick(fps)

def add_boids(boids, num_boids):
    for _ in range(num_boids):
        boids.add(Boid())

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Emergent flocking.')
    parser.add_argument('--geometry', metavar='WxH', type=str,
                        default=default_geometry, help='geometry of window')
    parser.add_argument('--number', dest='num_boids', default=default_boids,
                        help='number of boids to generate')
    args = parser.parse_args()
    main(args)