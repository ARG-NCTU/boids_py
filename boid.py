# boid.py
import pygame as pg
from random import uniform
from vehicle import Vehicle


class Boid(Vehicle):

    # CONFIG
    debug = False
    min_speed = .01
    max_speed = .2
    max_force = 1
    max_turn = 5
    perception = 60
    crowding = 15
    can_wrap = False
    edge_distance_pct = 5
    
    # 避障權重
    avoid_weight = 5.0 

    def __init__(self):
        Boid.set_boundary(Boid.edge_distance_pct)

        # Randomize starting position and velocity
        start_position = pg.math.Vector2(
            uniform(0, Boid.max_x),
            uniform(0, Boid.max_y))
        start_velocity = pg.math.Vector2(
            uniform(-1, 1) * Boid.max_speed,
            uniform(-1, 1) * Boid.max_speed)

        super().__init__(start_position, start_velocity,
                         Boid.min_speed, Boid.max_speed,
                         Boid.max_force, Boid.can_wrap)

        self.rect = self.image.get_rect(center=self.position)
        self.debug = Boid.debug

    def separation(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            dist = self.position.distance_to(boid.position)
            if dist < self.crowding:
                steering -= boid.position - self.position
        steering = self.clamp_force(steering)
        return steering

    def alignment(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            steering += boid.velocity
        steering /= len(boids)
        steering -= self.velocity
        steering = self.clamp_force(steering)
        return steering / 8

    def cohesion(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            steering += boid.position
        steering /= len(boids)
        steering -= self.position
        steering = self.clamp_force(steering)
        return steering / 100

    # --- 輔助：計算點到線段最近點 ---
    def get_closest_point_on_segment(self, p, a, b):
        ap = p - a
        ab = b - a
        ab_len_sq = ab.length_squared()
        if ab_len_sq == 0:
            return a
        t = ap.dot(ab) / ab_len_sq
        t = max(0, min(1, t))
        return a + ab * t

    # --- 修改後的避障邏輯 (Convex Hull) ---
    def avoid_obstacles(self, clusters):
        steering = pg.Vector2()
        
        for polygon in clusters:
            if len(polygon) < 2:
                # 單點處理
                if len(polygon) == 1:
                    diff = self.position - polygon[0]
                    if diff.length() < self.perception:
                         steering += diff.normalize() * self.max_force
                continue

            # 遍歷多邊形每一邊
            closest_dist_to_poly = float('inf')
            closest_point_on_poly = pg.Vector2()
            
            for i in range(len(polygon)):
                p1 = polygon[i]
                p2 = polygon[(i + 1) % len(polygon)] # 連回起點
                
                c_point = self.get_closest_point_on_segment(self.position, p1, p2)
                dist = self.position.distance_to(c_point)
                
                if dist < closest_dist_to_poly:
                    closest_dist_to_poly = dist
                    closest_point_on_poly = c_point

            # 產生避障力
            if closest_dist_to_poly < self.perception:
                diff = self.position - closest_point_on_poly
                
                if closest_dist_to_poly < 0.1:
                    # 極端接近時的反向力
                    diff = -self.velocity 
                    steering += diff.normalize() * self.max_force * 5
                else:
                    strength = (self.perception / closest_dist_to_poly) * 2
                    steering += diff.normalize() * strength
                    
                    # 切線力 (解決垂直撞牆)
                    if self.velocity.length() > 0:
                        angle_diff = self.velocity.angle_to(diff)
                        if abs(angle_diff) > 150:
                            tangent = diff.rotate(90)
                            steering += tangent * 3

        return steering

    def update(self, dt, boids, clusters=None):
        steering = pg.Vector2()

        # 0. 邊界檢查
        if not self.can_wrap:
            steering += self.avoid_edge()

        # 1. 避障力
        avoidance = pg.Vector2()
        if clusters:
            avoidance = self.avoid_obstacles(clusters)

        # 2. 優先級邏輯
        if avoidance.length() > 0.5: 
            steering += avoidance * self.avoid_weight
            neighbors = self.get_neighbors(boids)
            if neighbors:
                steering += self.separation(neighbors) * 2
        else:
            neighbors = self.get_neighbors(boids)
            if neighbors:
                separation = self.separation(neighbors)
                alignment = self.alignment(neighbors)
                cohesion = self.cohesion(neighbors)
                steering += separation + alignment + cohesion
        
        super().update(dt, steering)

    def get_neighbors(self, boids):
        neighbors = []
        for boid in boids:
            if boid != self:
                dist = self.position.distance_to(boid.position)
                if dist < self.perception:
                    neighbors.append(boid)
        return neighbors