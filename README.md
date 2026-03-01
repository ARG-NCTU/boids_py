# Boids

This repo reproduces Craig Reynolds' [Boids](https://www.red3d.com/cwr/boids/) using Python and `pygame`, with a focus on convex-hull-based obstacle avoidance.

## Requirements
* Python 3.8 or newer
* [pygame](https://www.pygame.org)

## Installation
Install dependencies with:

```bash
apt install python3-pygame
```

## Running
```bash
python3 main.py
```

To adjust the window size or number of boids, use the optional flags:

```bash
python3 main.py --geometry 1280x720 --number 200
```

`--geometry WxH` changes the window dimensions (width x height) and `--number` controls the starting flock size.

## Configuration
Edit the `CONFIG` block in `boid.py` to tune flocking behavior without diving into the update loop. Common knobs:

* `min_speed` / `max_speed` / `max_force` — clamp how quickly a boid can move and how hard it can steer.
* `max_turn` — limits abrupt heading changes applied by `Vehicle`.
* `perception` / `crowding` — govern how many neighbors influence separation, alignment, and cohesion forces.
* `avoid_weight`, `can_wrap`, `edge_distance_pct` — control obstacle avoidance strength and boundary behavior.

The obstacle pipeline in `main.py` exposes `connection_radius` plus helpers such as `add_obstacle_cluster`; lowering `connection_radius` forces tighter clusters while raising it lets more distant sprites merge into hulls.

## Controls
While the window has focus:

* `Q`: quit.
* `UP` / `DOWN`: add/remove boids (Shift increases the change from 10 to 100).
* `1` / `2`: halve/double the boids' `max_force`.
* `3` / `4`: shrink/grow `perception`.
* `5` / `6`: shrink/grow `crowding`.
* `7` / `8`: decrease/increase the obstacle cluster visual radius.
* `9` / `0`: decrease/increase `avoid_weight`.
* `[` / `]`: remove/add obstacle clusters.
* `D`: toggle per-boid debug drawing.
* `R`: respawn all boids (keeps count constant).

## Obstacles
Obstacles spawn in gaussian clusters (`add_obstacle_cluster`) and are grouped into convex hulls through `get_obstacle_clusters`. The hulls are rendered in cyan and passed to `Boid.avoid_obstacles()` so the flock steers away from walls and dense obstacle patches.