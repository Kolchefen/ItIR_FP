# Launch Guide

How to build and run the `turtlebot4_reactive_controller` package.

## Prerequisites

- ROS 2 workspace rooted at `~/final_proj`
- Map file at `/home/lill0017/final_proj/area_map/map_area.yaml`
- All terminals must source the workspace overlay:
  ```bash
  source install/setup.bash
  ```

## 1. Build

From `~/final_proj`:

```bash
colcon build --packages-select turtlebot4_reactive_controller --symlink-install
source install/setup.bash
```

`--symlink-install` lets you edit Python files in `src/` without rebuilding.

## 2. Launch Nav2 + localization + RViz (Terminal 1)

```bash
ros2 launch turtlebot4_reactive_controller bringup.launch.py \
    map:=/home/lill0017/final_proj/area_map/map_area.yaml
```

Optional launch args:

| Arg            | Default                                               | When to change                          |
| -------------- | ----------------------------------------------------- | --------------------------------------- |
| `map`          | `/home/lill0017/final_proj/area_map/map_area.yaml`    | Different map                           |
| `use_sim_time` | `false`                                               | Set `true` when running in simulation   |
| `use_rviz`     | `true`                                                | Set `false` on the Pi with no display   |

This spawns a single `nav2_container` with localization (map_server + amcl) and the full Nav2 stack (controller, planner, smoother, behaviors, bt_navigator, waypoint_follower, velocity_smoother) composed in-process.

## 3. Set the initial pose

In RViz, click **2D Pose Estimate** and click-drag where the robot actually is on the map. AMCL will not publish `/amcl_pose` until this is done, and the TSP executor blocks on that topic.

## 4. Run the TSP executor (Terminal 2)

```bash
ros2 run turtlebot4_reactive_controller tsp_executor
```

### Testing D* Lite without face detection

Set the face-check distance large enough that it never triggers:

```bash
ros2 run turtlebot4_reactive_controller tsp_executor \
    --ros-args -p face_check_distance_m:=500.0
```

The `/face_detected` subscription still exists but does nothing because the distance threshold is never crossed. No need to launch the face_detector node.

### What happens on startup

1. Waits for `/map` and `/amcl_pose`.
2. Builds an inflated obstacle grid (robot radius 0.18 m).
3. Runs D* Lite once per vertex (robot start + every waypoint) to populate a pairwise cost matrix.
4. Brute-forces 4! = 24 waypoint permutations to pick the shortest tour.
5. Sends each leg to Nav2's `/navigate_to_pose` action in order.

## 5. (Optional) Face detector (Terminal 3)

Only needed when you want the turn-around-and-beep interrupt to fire:

```bash
ros2 run turtlebot4_reactive_controller face_detector
```

## ROS parameters (tsp_executor)

| Parameter                | Default                | Purpose                                       |
| ------------------------ | ---------------------- | --------------------------------------------- |
| `face_check_distance_m`  | `500` (source default) | Meters between face-check interrupts          |
| `face_topic`             | `/face_detected`       | Bool topic from face_detector                 |
| `cmd_vel_topic`          | `/cmd_vel_unstamped`   | Create 3 Twist topic for the 180 rotation     |

## Troubleshooting

- **TSP executor hangs on "waiting for /map and /amcl_pose"** — you haven't set the 2D Pose Estimate in RViz yet, or the map file path is wrong.
- **"waypoint X maps to blocked or out-of-bounds cell"** — a waypoint in `waypoints.py` is inside an inflated obstacle. Either move the waypoint or lower `ROBOT_RADIUS_M`.
- **"no feasible tour"** — at least one leg is unreachable on the current map. Check that the map covers all waypoints and that the robot pose is in free space.
- **AMCL bond timeouts** — the bringup deliberately uses a single composed container to avoid this. If you see it, confirm `use_composition:=True` is being passed (it is, by default).
