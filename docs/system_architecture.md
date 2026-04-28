# System Architecture

Block diagram of the three decoupled layers and the ROS 2 topics/actions that
connect them. Solid arrows are publish/subscribe topics; the double-bordered
edge is the `NavigateToPose` action.

```mermaid
flowchart LR
    subgraph Perception["Perception layer"]
        OAKD[/"OAK-D camera<br/>(/color/preview/image)"/]
        FD["face_detector<br/><i>Haar cascade + debounce</i>"]
    end

    subgraph Planning["Planning layer"]
        TSP["tsp_executor<br/><i>D* Lite + TSP + face-check supervisor</i>"]
    end

    subgraph Execution["Execution layer (Nav2)"]
        MAP["map_server"]
        AMCL["amcl"]
        NAV2["controller / planner /<br/>smoother / bt_navigator"]
    end

    subgraph Robot["Create 3 base"]
        ODOM[/"wheel odometry"/]
        BASE["motor + speaker"]
    end

    OAKD -- "image_raw" --> FD
    FD -- "/face_detected (Bool)" --> TSP

    MAP -- "/map (OccupancyGrid)" --> TSP
    MAP -- "/map" --> AMCL
    MAP -- "/map" --> NAV2
    AMCL -- "/amcl_pose (PoseWithCovarianceStamped)" --> TSP
    AMCL -- "tf: map->odom" --> NAV2

    ODOM -- "/odom (Odometry)" --> TSP
    ODOM -- "/odom" --> NAV2
    ODOM -- "/odom" --> AMCL

    TSP == "/navigate_to_pose (action)" ==> NAV2
    NAV2 -- "/cmd_vel" --> BASE
    TSP -- "/cmd_vel_unstamped (Twist, 180° turn)" --> BASE
    TSP -- "/cmd_audio (AudioNoteVector, beep)" --> BASE
```

## Legend

| Edge                     | Direction                          | Purpose                                               |
| ------------------------ | ---------------------------------- | ----------------------------------------------------- |
| `/map`                   | map_server → tsp_executor, Nav2    | Static occupancy grid (latched, TRANSIENT_LOCAL)      |
| `/amcl_pose`             | amcl → tsp_executor                | Robot pose estimate in the `map` frame                |
| `/odom`                  | Create 3 → tsp_executor, Nav2, amcl| Wheel odometry; used for distance + yaw integration   |
| `/face_detected`         | face_detector → tsp_executor       | Debounced Bool flag                                   |
| `/navigate_to_pose`      | tsp_executor → Nav2 (action)       | One goal per leg of the tour                          |
| `/cmd_vel_unstamped`     | tsp_executor → Create 3            | Open-loop angular velocity during the 180° turn       |
| `/cmd_audio`             | tsp_executor → Create 3            | 880 Hz / 200 ms notes while waiting for a face        |
| `/cmd_vel`               | Nav2 → Create 3                    | Closed-loop velocity from the Nav2 controller server  |

## Legend

| Edge                     | Direction                          | Purpose                                               |
| ------------------------ | ---------------------------------- | ----------------------------------------------------- |
| `/map`                   | map_server → tsp_executor, Nav2    | Static occupancy grid (latched, TRANSIENT_LOCAL)      |
| `/amcl_pose`             | amcl → tsp_executor                | Robot pose estimate in the `map` frame                |
| `/odom`                  | Create 3 → tsp_executor, Nav2, amcl| Wheel odometry; used for distance + yaw integration   |
| `/face_detected`         | face_detector → tsp_executor       | Debounced Bool flag                                   |
| `/navigate_to_pose`      | tsp_executor → Nav2 (action)       | One goal per leg of the tour                          |
| `/cmd_vel_unstamped`     | tsp_executor → Create 3            | Open-loop angular velocity during the 180° turn       |
| `/cmd_audio`             | tsp_executor → Create 3            | 880 Hz / 200 ms notes while waiting for a face        |
| `/cmd_vel`               | Nav2 → Create 3                    | Closed-loop velocity from the Nav2 controller server  |

