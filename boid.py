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
    
    # --- 新增：避障權重 (由 2.0 提取出來變成變數) ---
    avoid_weight = 5.0 
    ###############

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

    # --- 修改：使用 self.avoid_weight ---
    def avoid_obstacles(self, clusters):
        steering = pg.Vector2()
        
        for rect in clusters:
            # 1. 建立一個稍微擴大的偵測區 (Inflate)
            # 這裡設定比 perception 大一點，確保船隻在還沒撞到前就開始有反應
            detect_box = rect.inflate(self.perception * 1.5, self.perception * 1.5)
            
            # 2. 檢查船隻是否進入了這個偵測區
            if detect_box.collidepoint(self.position):
                
                # --- 關鍵演算法：尋找矩形上距離我最近的點 (Clamping) ---
                # 這能確保斥力永遠是垂直於矩形表面的（最有效率的避讓方向）
                closest_x = max(rect.left, min(self.position.x, rect.right))
                closest_y = max(rect.top, min(self.position.y, rect.bottom))
                closest_point = pg.Vector2(closest_x, closest_y)
                
                # 計算我與最近點的向量 (這就是我要逃離的方向)
                diff = self.position - closest_point
                
                dist = diff.length()
                
                # 如果我已經在矩形內部 (dist == 0) 或者距離非常近
                if dist < 0.1:
                    # 緊急情況：隨機給一個強大斥力或是沿著目前速度反向
                    diff = -self.velocity 
                    diff.scale_to_length(self.max_force * 2)
                else:
                    # 距離越近，斥力越強 (指數級增強效果更好)
                    # 這裡使用 1/dist 的平方或是簡單的線性加權
                    force_magnitude = (1.0 / dist) * 50 
                    diff.scale_to_length(force_magnitude)
                
                steering += diff

        if steering.length() > 0:
            steering = self.clamp_force(steering)
            steering *= self.avoid_weight  # 乘上權重
            
        return steering

    def update(self, dt, boids, clusters=None): # 改名為 clusters 比較語意化
        steering = pg.Vector2()

        if not self.can_wrap:
            steering += self.avoid_edge()

        neighbors = self.get_neighbors(boids)
        if neighbors:
            separation = self.separation(neighbors)
            alignment = self.alignment(neighbors)
            cohesion = self.cohesion(neighbors)
            steering += separation + alignment + cohesion

        if clusters:
            avoidance = self.avoid_obstacles(clusters) # 傳入 clusters
            steering += avoidance

        super().update(dt, steering)
        
    def get_neighbors(self, boids):
        neighbors = []
        for boid in boids:
            if boid != self:
                dist = self.position.distance_to(boid.position)
                if dist < self.perception:
                    neighbors.append(boid)
        return neighbors