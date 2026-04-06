# Robocept Nav — Navigation and Decision Logic

ROS 2 navigation layer for the robocept robot. Consumes perception data and produces velocity commands. This is the "brain" that decides where and how the robot moves.

## Architecture

```
/robocept/lidar/scan ──────┐
/robocept/camera/* ────────┤
/robocept/detections ──────┤
                           ▼
                  ┌─────────────────┐
                  │  obstacle_avoid  │  reactive safety layer (always active)
                  │  waypoint_nav    │  goal-directed navigation
                  └────────┬────────┘
                           │
                           ▼
                       /cmd_vel
```

## Packages

| Package | Purpose |
|---|---|
| `robocept_nav` | Navigation nodes: obstacle avoidance, waypoint following |

## Nodes

### obstacle_avoider

Reactive obstacle avoidance using LiDAR scan data. Acts as a safety layer — overrides or modulates `/cmd_vel` to prevent collisions.

**Subscribes:**
- `/robocept/lidar/scan` (`sensor_msgs/LaserScan`)
- `/nav_cmd_vel` (`geometry_msgs/Twist`) — upstream velocity command (from waypoint nav or teleop)

**Publishes:**
- `/cmd_vel` (`geometry_msgs/Twist`) — safe velocity command sent to base driver

**Parameters:**
- `min_obstacle_distance` (0.35m) — stop if obstacle closer than this
- `slowdown_distance` (0.8m) — reduce speed when obstacle within this range
- `max_linear_vel` (0.5 m/s)
- `max_angular_vel` (1.5 rad/s)
- `scan_angle_front` (60 deg) — FOV to check for obstacles

### waypoint_nav

Simple waypoint follower using odometry. Drives to a sequence of (x, y, theta) goals.

**Subscribes:**
- `/odom` (`nav_msgs/Odometry`)

**Publishes:**
- `/nav_cmd_vel` (`geometry_msgs/Twist`) — velocity command (filtered by obstacle_avoider)

**Services:**
- `/robocept/nav/go_to` (`robocept_nav_msgs/GoTo`) — send a goal pose
- `/robocept/nav/stop` (`std_srvs/Empty`) — stop navigation

## Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/robocept_ws
colcon build --symlink-install --packages-select robocept_nav
source install/setup.bash
```

## Usage

```bash
# Obstacle avoidance only (with teleop)
ros2 launch robocept_nav obstacle_avoid.launch.py

# Full nav (waypoint + obstacle avoidance)
ros2 launch robocept_nav nav.launch.py
```
