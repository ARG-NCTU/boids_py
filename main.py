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
# 這是用來畫線的距離閾值，模擬 DBSCAN 的 Epsilon
connection_radius = 40.0 

class Obstacle(pg.sprite.Sprite):
    def __init__(self, position):
        super().__init__()
        self.image = pg.Surface((4, 4))
        self.image.fill(pg.Color('red'))
        self.rect = self.image.get_rect(center=position)
        self.position = pg.Vector2(position)

# 為了方便動態增減，將生成單一聚落的邏輯獨立出來
def add_obstacle_cluster(obstacles, width, height, points=20):
    cluster_x = uniform(50, width - 50)
    cluster_y = uniform(50, height - 50)
    for _ in range(points):
        ox = gauss(cluster_x, 30)
        oy = gauss(cluster_y, 30)
        ox = max(0, min(ox, width))
        oy = max(0, min(oy, height))
        obstacles.add(Obstacle((ox, oy)))
# main.py

def get_obstacle_clusters(obstacles, radius):
    """
    將障礙物分群並計算 Bounding Box (BBOX)
    回傳: 一個包含 pg.Rect 物件的列表
    """
    sprites = obstacles.sprites()
    if not sprites:
        return []

    clusters = []
    visited = set()

    for sprite in sprites:
        if sprite in visited:
            continue

        # 開始一個新的 cluster 搜尋 (BFS/Flood Fill)
        current_cluster = [sprite]
        visited.add(sprite)
        queue = [sprite]

        while queue:
            current = queue.pop(0)
            
            # 找所有還沒被訪問過的鄰居
            for other in sprites:
                if other not in visited:
                    if current.position.distance_to(other.position) <= radius:
                        visited.add(other)
                        current_cluster.append(other)
                        queue.append(other)

        # 計算這個 cluster 的 BBOX
        min_x = min(s.position.x for s in current_cluster)
        max_x = max(s.position.x for s in current_cluster)
        min_y = min(s.position.y for s in current_cluster)
        max_y = max(s.position.y for s in current_cluster)

        # 建立 Rect (x, y, width, height)
        # 稍微向外擴張一點 (margin)，讓框框包住點而不是切過點中心
        margin = 10
        rect = pg.Rect(min_x - margin, min_y - margin, 
                       (max_x - min_x) + margin*2, (max_y - min_y) + margin*2)
        clusters.append(rect)

    return clusters
def update(dt, boids, obstacles, clusters,window_size):
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
            
            # --- Boids 原有控制 ---
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
            
            # --- 新增：互動式障礙物控制 ---
            
            # 7/8: 調整視覺連結距離 (Cluster Distance)
            elif event.key == pg.K_7:
                connection_radius = max(5, connection_radius - 5)
                print(f"Cluster Visual Radius: {connection_radius}")
            elif event.key == pg.K_8:
                connection_radius += 5
                print(f"Cluster Visual Radius: {connection_radius}")

            # 9/0: 調整避障權重 (Avoidance Weight)
            elif event.key == pg.K_9:
                for b in boids: b.avoid_weight = max(0.1, b.avoid_weight - 0.5)
                # 因為是類別變數，其實改一個全部都會變，但這樣寫保險
                Boid.avoid_weight = boids.sprites()[0].avoid_weight
                print(f"Obstacle Avoid Weight: {Boid.avoid_weight}")
            elif event.key == pg.K_0:
                for b in boids: b.avoid_weight += 0.5
                Boid.avoid_weight = boids.sprites()[0].avoid_weight
                print(f"Obstacle Avoid Weight: {Boid.avoid_weight}")

            # [ / ]: 減少 / 增加 障礙物聚落
            elif event.key == pg.K_LEFTBRACKET: # [ 鍵
                # 隨機移除 20 個點 (約一個聚落)
                sprites = obstacles.sprites()
                if len(sprites) > 0:
                    obstacles.remove(sprites[:20])
                print(f"Obstacles count: {len(obstacles)}")
            elif event.key == pg.K_RIGHTBRACKET: # ] 鍵
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
        # 2. 將 calculated clusters 傳給 boid，而不是原始 obstacles
        b.update(dt, boids, clusters)

# 新增：繪製連結線 (模擬 DBSCAN 可視化)
def draw_connections(screen, obstacles):
    # 建立一個透明圖層畫線，看起來比較高級
    overlay = pg.Surface(screen.get_size(), pg.SRCALPHA)
    
    # 取得所有障礙物列表
    obs_list = obstacles.sprites()
    
    # 用青色 (Cyan) 代表雷達/Lidar 掃描線的感覺
    line_color = (0, 255, 255, 50) # 最後一個數字是 Alpha 透明度 (0-255)
    
    # 雙重迴圈檢查距離 (效能注意：點太多會慢，但幾百個點沒問題)
    for i, obs1 in enumerate(obs_list):
        for j in range(i + 1, len(obs_list)):
            obs2 = obs_list[j]
            dist = obs1.position.distance_to(obs2.position)
            
            # 如果距離小於設定的半徑，就畫線
            if dist < connection_radius:
                pg.draw.line(overlay, line_color, obs1.position, obs2.position, 1)
    
    screen.blit(overlay, (0, 0))

def draw(screen, background, boids, obstacles, clusters): # <--- 新增 clusters 參數
    # 1. 清空畫面
    screen.blit(background, (0, 0))

    # 2. 畫出 BBOX (取代原本的連線)
    for rect in clusters:
        # 畫一個青色的矩形框，寬度 1
        pg.draw.rect(screen, (0, 255, 255), rect, 1)
        
        # (選用) 畫出半透明的填充，看起來更有高科技感
        s = pg.Surface((rect.width, rect.height), pg.SRCALPHA)
        s.fill((0, 255, 255, 30)) # 30 是透明度
        screen.blit(s, (rect.x, rect.y))

    # 3. 畫障礙點
    obstacles.draw(screen)
    
    # 4. 畫船
    boids.draw(screen)

    pg.display.flip()

def main(args):
    pg.init()
    pg.event.set_allowed([pg.QUIT, pg.KEYDOWN, pg.KEYUP])
    fps = 60.0
    fpsClock = pg.time.Clock()
    pg.display.set_caption("BOIDS - Obstacle Interaction")
    
    window_width, window_height = [int(x) for x in args.geometry.split("x")]
    flags = DOUBLEBUF
    screen = pg.display.set_mode((window_width, window_height), flags)
    screen.set_alpha(None)
    background = pg.Surface(screen.get_size()).convert()
    background.fill(pg.Color('black'))

    boids = pg.sprite.RenderUpdates()
    add_boids(boids, args.num_boids)

    # 初始化障礙物 (RenderUpdates)
    obstacles = pg.sprite.RenderUpdates()
    # 初始生成 5 個聚落
    for _ in range(5):
        add_obstacle_cluster(obstacles, window_width, window_height)

    dt = 1/fps

    while True:
        # 1. 每一幀先算出 Cluster
        clusters = get_obstacle_clusters(obstacles, connection_radius)

        # 2. 傳給 update (讓船知道要避開方塊)
        # 注意：這裡我們改了 update 的定義，現在不需要傳 obstacles 給船了，傳 clusters 就好
        # 但為了按鍵控制生成障礙物，update 裡面還是可能用到 obstacles 變數
        update(dt, boids, obstacles, clusters,(window_width, window_height))
        
        # 修正：實際上我們的 update 迴圈裡是呼叫 b.update(dt, boids, clusters)
        # 所以要在這裡做一個小調整，讓 update 函式能拿到 clusters
        
        # 3. 傳給 draw (畫出方塊)
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