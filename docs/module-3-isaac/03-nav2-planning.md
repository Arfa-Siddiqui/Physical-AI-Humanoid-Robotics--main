---
sidebar_position: 3
---

# Nav2: Autonomous Navigation and Path Planning

Nav2 (Navigation2) is the ROS 2 navigation stack, enabling robots to autonomously navigate from point A to B while avoiding obstacles. This chapter integrates Nav2 with Isaac Sim and Isaac ROS for complete autonomous navigation.

## Nav2 Architecture

Nav2 uses a **behavior tree** architecture with pluggable components:

```
User Goal → Behavior Tree → Planner → Controller → Robot
                ↑              ↓          ↓
            Costmaps ← Sensors (LiDAR, Camera)
```

**Key Components:**
- **Planner**: Computes global path (A* algorithm)
- **Controller**: Follows path (DWB, TEB, or MPPI)
- **Costmap**: 2D grid (free space, obstacles, inflation)
- **Behavior Server**: Executes recovery behaviors (backup, spin)

## Installing Nav2

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
```

**Verify:**

```bash
ros2 pkg list | grep nav2
# Should list 30+ packages
```

## Configuring Nav2 for Humanoid Robots

### Navigation Parameters

Create `nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false  # Dijkstra (more robust for complex maps)

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5  # Max forward speed (m/s)
      min_vel_y: 0.0
      max_vel_y: 0.0  # Non-holonomic (differential drive)
      max_vel_theta: 1.0  # Max rotation speed (rad/s)
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      acc_lim_x: 1.0
      acc_lim_y: 0.0
      acc_lim_theta: 2.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 10
      height: 10
      resolution: 0.05
      robot_radius: 0.3  # Humanoid footprint radius
      plugins: ["obstacle_layer", "inflation_layer"]

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.3
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

## Creating a Map with SLAM

Before navigation, create a map using SLAM.

### Option 1: SLAM Toolbox (2D)

```bash
sudo apt install ros-humble-slam-toolbox -y
```

**Launch SLAM:**

```bash
ros2 launch slam_toolbox online_async_launch.py
```

**Drive Robot:**

Use teleop to explore environment:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Save Map:**

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

Generates `my_map.yaml` and `my_map.pgm`.

### Option 2: Isaac ROS Visual SLAM (3D)

For 3D mapping:

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

Outputs pose estimates for 2D projection into Nav2.

## Launching Nav2

**Launch File** (`nav2_launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_nav')
    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[{'yaml_filename': map_file}]
        ),

        # Nav2 Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{'autostart': True, 'node_names': ['map_server']}]
        ),

        # Nav2 Stack
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            parameters=[params_file]
        )
    ])
```

**Run:**

```bash
ros2 launch my_robot_nav nav2_launch.py
```

## Sending Navigation Goals

### RViz Interface

```bash
ros2 run rviz2 rviz2 -d `ros2 pkg prefix nav2_bringup`/share/nav2_bringup/rviz/nav2_default_view.rviz
```

1. **2D Pose Estimate**: Click toolbar button → click/drag on map to set initial pose
2. **Nav2 Goal**: Click toolbar button → click/drag on map to set goal

Robot plans path and navigates.

### Programmatic Goal (Python)

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = NavGoalSender()
    future = node.send_goal(5.0, 3.0, 0.0)  # Goal: (5, 3, facing east)
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info('Goal sent!')
    rclpy.shutdown()
```

## Dynamic Obstacle Avoidance

Nav2 continuously updates costmaps from sensor data.

**Configure Obstacle Layer** (`nav2_params.yaml`):

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  observation_sources: scan
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: True
    marking: True
    data_type: "LaserScan"
```

As LiDAR detects obstacles, they appear in costmap, and planner reroutes.

## Recovery Behaviors

When stuck, Nav2 executes recovery behaviors:

1. **Clear Costmap**: Remove transient obstacles
2. **Spin**: Rotate 360° to reassess
3. **Backup**: Move backward
4. **Wait**: Pause, hoping obstacle moves

**Configure in Behavior Tree:**

```yaml
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
```

## Integrating with Isaac Sim

### Launch Isaac Sim Scene

In Isaac Sim:
1. Load warehouse world
2. Spawn humanoid robot
3. Enable ROS 2 Bridge

### Bridge Odometry and TF

Isaac Sim publishes:
- `/odom` (nav_msgs/Odometry)
- `/tf` (TF tree: map → odom → base_link)

Nav2 consumes these for localization.

### Test Full Stack

```bash
# Terminal 1: Isaac Sim (already running)

# Terminal 2: Nav2
ros2 launch my_robot_nav nav2_launch.py use_sim_time:=True

# Terminal 3: RViz
ros2 run rviz2 rviz2 -d nav2_config.rviz
```

Set goal in RViz. Robot navigates in Isaac Sim.

## Tuning for Performance

### Planner Frequency

Increase for faster replanning:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0  # Replan 10× per second
```

### Costmap Resolution

Finer resolution (0.02m) improves accuracy but increases computation:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      resolution: 0.02  # 2cm per cell
```

**Trade-Off:** 0.05m is good for most robots; 0.02m for precise manipulation.

## Conclusion

Nav2 transforms a robot from teleoperated to autonomous. By integrating SLAM, path planning, and dynamic obstacle avoidance, humanoid robots can navigate complex environments safely. Combined with Isaac ROS perception and Isaac Sim testing, you have a complete autonomous navigation pipeline. In the next module, we'll add Vision-Language-Action capabilities for natural language task specification.

## References

- Macenski, S., et al. (2020). "Robot Operating System 2: Design, architecture, and uses in the wild." *Science Robotics*, 5(47).
- Macenski, S. (2024). "Nav2 Documentation." Retrieved from https://navigation.ros.org/
