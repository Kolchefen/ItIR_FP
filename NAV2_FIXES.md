# Nav2 Bringup Fixes

Fixes applied to get the default Nav2 stack working with the TurtleBot 4 and a pre-made map.

## 1. Config files not installed (Critical)

**Problem:** The `config/` directory (containing `nav2_params.yaml`) was added to the project
but `setup.py` had not been rebuilt. The `colcon build` install tree had no `config/` directory,
so the launch file's `FindPackageShare(...)` + `config/nav2_params.yaml` resolved to a
non-existent path. Nav2 either failed to start or used built-in defaults that don't match the
TurtleBot 4.

**Files changed:** `setup.py` (already had the correct `data_files` entry; just needed a rebuild).

**Fix:** Rebuilt with:

```bash
cd ~/final_proj
colcon build --packages-select turtlebot4_reactive_controller
source install/setup.bash
```

After the rebuild, `nav2_params.yaml` is present at:

```
install/turtlebot4_reactive_controller/share/turtlebot4_reactive_controller/config/nav2_params.yaml
```

## 2. Missing default for `map` launch argument

**Problem:** The `map` launch argument in `bringup.launch.py` had no `default_value`.
Forgetting to pass `map:=<path>` on the command line caused the launch to fail immediately.

**File changed:** `launch/bringup.launch.py`

**Fix:** Added a default value pointing to the pre-made map:

```python
map_arg = DeclareLaunchArgument(
    'map',
    default_value='/home/lill0017/final_proj/area_map/map_area.yaml',
    description='Full path to map yaml file',
)
```

The map can still be overridden at launch time:

```bash
ros2 launch turtlebot4_reactive_controller bringup.launch.py map:=/other/map.yaml
```

## 3. Inconsistent `base_frame_id` in AMCL

**Problem:** AMCL was configured with `base_frame_id: "base_footprint"` while every other
Nav2 node (`bt_navigator`, costmaps, `behavior_server`) used `base_link`. The TurtleBot 4
TF tree contains both frames, but the mismatch could cause transform lookup failures or
subtle localization drift.

**File changed:** `config/nav2_params.yaml`

**Fix:** Changed AMCL's `base_frame_id` from `"base_footprint"` to `"base_link"` so all
Nav2 components reference the same frame.

## How to launch after these fixes

```bash
cd ~/final_proj
source install/setup.bash
ros2 launch turtlebot4_reactive_controller bringup.launch.py
```

This will start AMCL (localization), the full Nav2 stack, and RViz2 using the pre-made map
at `~/final_proj/area_map/map_area.yaml`.

Remember to set an initial pose estimate in RViz2 (use the "2D Pose Estimate" button) so
AMCL can localize the robot on the map before sending navigation goals.
