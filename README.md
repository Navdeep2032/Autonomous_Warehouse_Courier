# Autonomous Warehouse Courier — ROS 2 AMR

A small differential-drive robot that autonomously navigates a simulated
warehouse, visiting three named waypoints in sequence: **PICKUP → DROPOFF
→ DOCK**, using Nav2, AMCL, and slam_toolbox.

---

## Platform

ROS 2 Humble · Gazebo Classic · Nav2 · slam_toolbox · AMCL ·
nav2_simple_commander

---

## File Map

| File                                 | Purpose                                                                                                                                                                                                                                                     |
| ------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `urdf/robot.urdf.xacro`              | Differential-drive robot description — box chassis, two driven wheels, passive caster, 360° lidar, Gazebo diff-drive + ray sensor plugins                                                                                                                   |
| `worlds/warehouse.world`             | Gazebo SDF world — 10×10 m warehouse with outer walls, a narrow corridor, three shelves, two box obstacles, and three coloured floor zone markers (PICKUP/DROPOFF/DOCK)                                                                                     |
| `config/nav2_params.yaml`            | Full Nav2 parameter set — AMCL, costmaps (local + global), DWB local planner, NavFn global planner, behavior server (spin/backup/wait recoveries), BT navigator, waypoint follower                                                                          |
| `maps/warehouse.pgm`                 | Saved occupancy grid map image (generated via `slam_toolbox` + `map_saver_cli`)                                                                                                                                                                             |
| `maps/warehouse.yaml`                | Map metadata — resolution, origin, occupancy thresholds                                                                                                                                                                                                     |
| `launch/warehouse_bringup.launch.py` | Single launch file: Gazebo → robot spawn → Nav2 bringup → RViz2, in correct timed order                                                                                                                                                                     |
| `warehouse_courier/mission.py`       | Mission script using `nav2_simple_commander.BasicNavigator` — visits PICKUP → DROPOFF → DOCK in sequence, prints live distance/ETA telemetry on one overwriting line, handles `TaskResult.FAILED`/`CANCELED` gracefully, lifecycle managed in `try/finally` |
| `warehouse_courier/auto_mapper.py`   | Optional auto-drive helper script — drives a clean, repeatable straight-then-rotate loop around the warehouse for higher-quality SLAM mapping (no manual teleop drift)                                                                                      |
| `package.xml`                        | ROS 2 package manifest — declares all build/runtime dependencies (`rclpy`, `nav2_simple_commander`, `nav2_bringup`, `gazebo_ros`, `slam_toolbox`, etc.)                                                                                                     |
| `setup.py` / `setup.cfg`             | ament_python build configuration — installs launch/urdf/worlds/config/maps into the package share directory, registers the `mission` and `auto_mapper` console scripts                                                                                      |

---

## Build

```bash
cd ~/autonomous_warehouse_courier
colcon build --symlink-install
source install/setup.bash
```

---

## Step 1 — SLAM Mapping (run once to generate the map)

**Terminal 1 — Gazebo + warehouse world:**

```bash
ros2 launch gazebo_ros gazebo.launch.py \
  world:=$(ros2 pkg prefix warehouse_courier)/share/warehouse_courier/worlds/warehouse.world
```

**Terminal 2 — Robot State Publisher:**

```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix warehouse_courier)/share/warehouse_courier/urdf/robot.urdf.xacro)" \
  -p use_sim_time:=true
```

**Terminal 3 — Spawn the robot:**

```bash
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description -entity courier -x -4.0 -y -4.0 -z 0.1
```

**Terminal 4 — slam_toolbox:**

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**Terminal 5 — Drive manually or with the auto-mapper helper:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# or
ros2 run warehouse_courier auto_mapper
```

**Terminal 6 — Save the map once it looks complete in RViz:**

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/autonomous_warehouse_courier/src/warehouse_courier/maps/warehouse
```

Then rebuild so the saved map is installed into the package share directory:

```bash
cd ~/autonomous_warehouse_courier
colcon build --symlink-install
source install/setup.bash
```

---

## Step 2 — Full Autonomous Run

**Launch command (brings up Gazebo + robot spawn + Nav2 + RViz2, in order):**

```bash
ros2 launch warehouse_courier warehouse_bringup.launch.py
```

In RViz2, click **2D Pose Estimate**, click the robot's actual position on
the map (its Gazebo spawn point, the DOCK zone), and drag in its facing
direction to set the initial AMCL pose. The particle cloud should converge
within ~5 seconds.

**Run the mission (new terminal):**

```bash
ros2 run warehouse_courier mission
```

The robot will autonomously drive **PICKUP → DROPOFF → DOCK** in one
continuous run, printing live distance and ETA telemetry to the terminal.

---

## Waypoints

| Name    | Coordinates (map frame) | Heading |
| ------- | ----------------------- | ------- |
| PICKUP  | (-2.0, 3.5)             | 0°      |
| DROPOFF | (3.5, 3.5)              | 90°     |
| DOCK    | (-4.0, -4.0)            | 180°    |

---

## Folder Structure

```
autonomous_warehouse_courier/
├── README.md
└── src/
    └── warehouse_courier/
        ├── config/
        │   └── nav2_params.yaml              # AMCL, costmap, planner, controller tuning
        ├── launch/
        │   └── warehouse_bringup.launch.py   # Gazebo + spawn + Nav2 + RViz, one command
        ├── maps/
        │   ├── warehouse.pgm                 # saved SLAM map (occupancy grid image)
        │   └── warehouse.yaml                # map metadata (resolution, origin)
        ├── urdf/
        │   └── robot.urdf.xacro              # differential-drive robot + lidar
        ├── worlds/
        │   └── warehouse.world               # Gazebo warehouse with corridor + shelves
        ├── warehouse_courier/
        │   ├── __init__.py
        │   └── mission.py                    # PICKUP → DROPOFF → DOCK mission script
        ├── resource/
        │   └── warehouse_courier
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

## Notes

- `behavior_server` plugin names use the `nav2_behaviors/Spin`,
  `nav2_behaviors/BackUp`, `nav2_behaviors/Wait` (slash-separated pluginlib
  format), with the **backup** behavior key matching the default
  Nav2 behavior-tree XML's expected action server name (no underscore).
- `planner_server` uses `nav2_navfn_planner/NavfnPlanner` (slash format);
  all other Nav2 plugins in this configuration use the `::` C++ namespace
  format, matching this installation's pluginlib declarations
  (Nav2 1.1.20 / ROS 2 Humble).
- Wheel friction (`mu1`/`mu2`), contact stiffness/damping (`kp`/`kd`), and
  reduced `max_wheel_torque`/`max_wheel_acceleration` were tuned in the
  URDF to minimise chassis wobble during acceleration/deceleration, which
  otherwise degrades SLAM scan-matching quality.
