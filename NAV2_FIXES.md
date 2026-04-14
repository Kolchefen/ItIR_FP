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

## 4. MPPI controller overloaded — "Failed to make progress"

**Problem:** The controller loop was missing its 20 Hz target (running at ~16 Hz), causing
delayed velocity commands. The robot could not satisfy the `SimpleProgressChecker`'s
requirement of moving 0.5 m within 10 s, so navigation goals were repeatedly aborted with
`Failed to make progress`. The recovery spin behavior also failed because the robot was
effectively stuck in an abort/retry loop.

Root cause: the default MPPI parameters (`batch_size: 2000`, `time_steps: 56`) are too
computationally expensive for the available CPU, and the progress checker thresholds were
too tight for a slow-starting robot.

**Files changed:** `config/nav2_params.yaml`

**Fix:** Three changes:

1. Lowered `controller_frequency` from 20 Hz to 10 Hz so the controller isn't constantly
   missing its deadline:
   ```yaml
   controller_server:
     ros__parameters:
       controller_frequency: 10.0
   ```

   MPPI also requires `model_dt >= controller_period`, so `model_dt` was raised from 0.05
   to 0.1 to match the new 10 Hz period. Without this, `controller_server` fails to
   configure with the error `Controller period more then model dt`.
   ```yaml
   FollowPath:
     model_dt: 0.1   # was 0.05
   ```

2. Relaxed the progress checker so temporary slowdowns don't abort the goal:
   ```yaml
   progress_checker:
     required_movement_radius: 0.3   # was 0.5
     movement_time_allowance: 30.0   # was 10.0
   ```

**Note:** Initially also lowered `batch_size` (2000 → 600) and `time_steps` (56 → 30) for
MPPI, but this caused `controller_server` to error during the `configure` lifecycle
transition — MPPI silently rejected the smaller values. Reverted to the original MPPI
numbers; keeping the lower `controller_frequency` and relaxed progress checker is enough.

## 5. Intermittent map loading — race condition on startup

**Problem:** The map loaded inconsistently across launches. The lifecycle manager was
activating AMCL and costmap nodes before `map_server` had finished publishing the `/map`
topic, so those nodes silently missed the latched map message.

**Files changed:** `launch/bringup.launch.py`, `config/nav2_params.yaml`

**Fix 1 — Startup delay in launch file:**
Wrapped the Nav2 bringup in a `TimerAction` (5 s delay) and RViz2 in a second
`TimerAction` (8 s delay). This gives `map_server` time to come up and publish before
the lifecycle manager tries to activate dependent nodes, and ensures RViz2 connects to
Nav2 only after its services are available.

```python
TimerAction(period=5.0, actions=[nav2_bringup]),
TimerAction(period=8.0, actions=[rviz_launch]),
```

**Fix 2 — Lifecycle manager bond timeout:**
The default `bond_timeout` of 4.0 s is too short on this hardware — `map_server` was
declared unreachable during startup with:
`Server map_server was unable to be reached after 4.00s by bond`.
Raised to 10.0 s for both lifecycle managers in `nav2_params.yaml`:

```yaml
lifecycle_manager_localization:
  ros__parameters:
    autostart: true
    bond_timeout: 10.0
    node_names: [map_server, amcl]

lifecycle_manager_navigation:
  ros__parameters:
    autostart: true
    bond_timeout: 10.0
    node_names: [controller_server, planner_server, behavior_server,
                 bt_navigator, waypoint_follower, smoother_server,
                 velocity_smoother]
```

**Note:** An earlier attempt used `bond_timeout: 20.0` and
`attempt_respawn_reconnection: false`. The 20 s timeout flooded DDS with heartbeat
traffic (`failed to send response to is_active (timeout)` warnings). 10 s is a
middle ground — long enough for slow startup, short enough to avoid queue buildup.

## 6. Localization lifecycle manager hangs after `map_server` configure

**Problem:** `map_server` loaded the map successfully, but AMCL was never
configured. The console showed:

```
[map_server] failed to send response to /map_server/change_state (timeout):
client will not receive response, at ./src/rmw_response.cpp:153
```

immediately after `Configuring map_server`, and no `Configuring amcl` line ever
appeared. On shutdown `lifecycle_manager_localization` had to be SIGKILL'd —
classic symptom of it still waiting on a service response it will never get.

Because AMCL never activated, no `map → odom` TF was published, so
`global_costmap` spun forever on `Invalid frame ID "map" passed to canTransform`
and nothing in the navigation stack worked.

Root cause: FastDDS (Jazzy's default RMW) intermittently drops or times out the
service response carrying the `configure` transition result. The lifecycle
manager's default `service_timeout` is too short to survive this on this
hardware, and the 5 s `TimerAction` delay was masking/prolonging the problem by
pushing all of Nav2's startup into the same 1 s window.

**Files changed:** `config/nav2_params.yaml`, `launch/bringup.launch.py`

**Fix 1 — Raise `service_timeout` on both lifecycle managers** so a slow
response from `change_state` / `get_state` doesn't permanently wedge the
bring-up sequence:

```yaml
lifecycle_manager_localization:
  ros__parameters:
    autostart: true
    bond_timeout: 10.0
    service_timeout: 10.0
    node_names: [map_server, amcl]

lifecycle_manager_navigation:
  ros__parameters:
    autostart: true
    bond_timeout: 10.0
    service_timeout: 10.0
    node_names: [controller_server, planner_server, behavior_server,
                 bt_navigator, waypoint_follower, smoother_server,
                 velocity_smoother]
```

**Fix 2 — Drop the 5 s `TimerAction` on Nav2 bringup.** The Nav2 lifecycle
manager already sequences startup correctly; the extra delay only compresses
node startup into a tighter window and makes DDS queue pressure worse. RViz2
still gets a short delay so its Nav2 panel finds active services on connect:

```python
return LaunchDescription([
    use_sim_time_arg,
    map_arg,
    nav2_bringup,
    TimerAction(period=3.0, actions=[rviz_launch]),
])
```

**Note:** The proper fix is to switch the RMW to Cyclone
(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`), which doesn't exhibit this
`failed to send response` behavior. Cyclone is unavailable in this environment,
so the two changes above are a workaround that keeps FastDDS.

**Note:** The `node_names` overrides in this file are effectively ignored by
`nav2_bringup`'s `bringup_launch.py` — the console log shows
`lifecycle_manager_navigation` also managing `route_server`,
`collision_monitor`, and `docking_server`, which come from nav2_bringup's
defaults, not this file. The `autostart`, `bond_timeout`, and `service_timeout`
params do propagate, which is what matters for this fix.

## 7. Lifecycle manager params ignored → bond timeout, `bt_navigator` never active

**Problem:** After fixes 1–6, `map_server`, `amcl`, `controller_server`, and
`smoother_server` all came up, but `planner_server` and `bt_navigator` never
reached the ACTIVE state, so every RViz goal was rejected with:

```
[bt_navigator] Action server is inactive. Rejecting the goal.
```

The console showed:

```
[lifecycle_manager_navigation] Server controller_server was unable to be
reached after 4.00s by bond. This server may be misconfigured.
[lifecycle_manager_navigation] Failed to bring up all requested nodes.
Aborting bringup.
```

The `4.00s` value is the hardcoded default `bond_timeout` — even though
`nav2_params.yaml` set `bond_timeout: 10.0`.

**Root cause:** `/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py`
(and the matching `localization_launch.py`) launches the lifecycle managers
like this:

```python
Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
)
```

`params_file` is **not** in that `parameters=` list. Every
`lifecycle_manager_*` setting in our `nav2_params.yaml` (bond_timeout,
service_timeout, node_names, etc.) was being silently ignored, and the
manager always used its compiled default `bond_timeout: 4.0`. On this
hardware, FastDDS reliably misses that 4 s window when forming the
`controller_server` bond — so navigation bringup aborted before it ever got
to `planner_server` or `bt_navigator`. That also invalidated fix #5's claim
that the 10 s `bond_timeout` in the yaml was doing anything.

**Files changed:** `launch/bringup.launch.py` (full rewrite),
`config/nav2_params.yaml` (cleanup of dead sections).

**Fix — stop using `nav2_bringup/bringup_launch.py`**. `bringup.launch.py`
now launches every Nav2 node directly as `Node(...)`, and declares both
lifecycle managers with parameters set as a dict so they actually take
effect:

```python
lifecycle_manager_params = {
    'autostart': True,
    'bond_timeout': 10.0,
    'service_timeout': 10.0,
}

Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    parameters=[{
        **lifecycle_manager_params,
        'node_names': navigation_lifecycle_nodes,
    }],
    ...
),
```

The nodes brought up directly are: `map_server`, `amcl`, `controller_server`,
`smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`,
`waypoint_follower`, `velocity_smoother`, plus the two lifecycle managers.
Remappings follow nav2_bringup's conventions (`/tf → tf`, `/tf_static →
tf_static`, `cmd_vel → cmd_vel_nav` on the velocity-producing nodes).

RViz2 is still launched via `nav2_bringup/launch/rviz_launch.py` with the
same 3 s `TimerAction` delay so its Nav2 panel sees active services.

### 7a. Dropped servers: `route_server`, `collision_monitor`, `docking_server`

These three are not needed for this project (D\* Lite planning + goal-pose
navigation from a random start) and each one:

- adds an extra node to the navigation lifecycle chain, so bond formation
  has to clear one more hurdle during bringup under a slow RMW;
- adds more DDS pub/sub traffic competing for time during the exact window
  where `lifecycle_manager_navigation` is waiting on bonds.

Dropping them shortens the navigation lifecycle list to 7 nodes and removes
the corresponding bond traffic. Their parameter blocks
(`route_server:`, `collision_monitor:`, `docking_server:`) were removed from
`nav2_params.yaml` to keep the file honest.

### 7b. velocity_smoother → cmd_vel remap

With `collision_monitor` removed, nothing was bridging
`velocity_smoother`'s output (`cmd_vel_smoothed`) to the robot's `cmd_vel`
topic. The launch file now adds an extra remap on `velocity_smoother`:

```python
remappings=tf_remaps + nav_cmd_remap + [
    ('cmd_vel_smoothed', 'cmd_vel'),
],
```

so the smoothed command is published directly to `cmd_vel`, which is what
the TurtleBot 4 subscribes to.

### 7c. Dead lifecycle_manager blocks removed from nav2_params.yaml

Because the lifecycle manager params are now set directly on the Node, the
top-of-file `lifecycle_manager_localization:` / `lifecycle_manager_navigation:`
blocks were dead weight (and had been dead weight all along). They were
replaced by a short comment explaining where the real values live.
