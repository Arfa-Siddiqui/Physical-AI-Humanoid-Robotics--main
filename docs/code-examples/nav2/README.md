# Nav2 Configuration Files

ROS 2 Nav2 (Navigation2) configurations for autonomous navigation with humanoid robots.

## Prerequisites

- ROS 2 Humble
- Nav2 installed:
  ```bash
  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
  ```
- Robot with:
  - Odometry (`/odom` topic)
  - Laser scan (`/scan` topic) or depth camera
  - TF tree properly configured

## Files

### 1. `nav2_params.yaml`
Complete Nav2 parameter configuration for humanoid robot navigation

**Configured components:**
- **Behavior Tree Navigator**: High-level navigation logic
- **Controller Server**: Local trajectory tracking (DWB)
- **Planner Server**: Global path planning (NavFn with Dijkstra)
- **Smoother Server**: Path smoothing for better trajectories
- **Behavior Server**: Recovery behaviors (spin, backup, wait)
- **Local Costmap**: Rolling window for local obstacles (3m x 3m)
- **Global Costmap**: Full map for global planning
- **Velocity Smoother**: Smooth acceleration/deceleration

**Key parameters for humanoid:**
- `robot_radius: 0.3` - Conservative footprint (30cm)
- `max_vel_x: 0.5` m/s - Safe walking speed
- `max_vel_theta: 1.0` rad/s - Turning speed
- `xy_goal_tolerance: 0.15` m - Position accuracy
- `yaw_goal_tolerance: 0.25` rad - Orientation accuracy

## Usage

### Basic Navigation Launch

```bash
# Launch Nav2 with custom params
ros2 launch nav2_bringup navigation_launch.py params_file:=/path/to/nav2_params.yaml

# Or include in your own launch file:
```

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('your_package')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            parameters=[params_file],
            output='screen'
        )
    ])
```

### Full Navigation Stack

Terminal setup for complete navigation:

```bash
# Terminal 1: Start simulator (Gazebo or Isaac Sim)
ros2 launch your_package simulation.launch.py

# Terminal 2: Start SLAM or localization
# Option A: SLAM (create map while navigating)
ros2 launch slam_toolbox online_async_launch.py

# Option B: Localization (use existing map)
ros2 launch nav2_bringup localization_launch.py map:=my_map.yaml

# Terminal 3: Start Nav2
ros2 launch nav2_bringup navigation_launch.py params_file:=nav2_params.yaml

# Terminal 4: Start RViz for visualization
ros2 launch nav2_bringup rviz_launch.py
```

### Setting Navigation Goals

**Via RViz:**
1. Open RViz with Nav2 config
2. Click "2D Goal Pose" button
3. Click and drag on map to set goal pose

**Via Command Line:**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, \
   pose: {position: {x: 2.0, y: 1.0, z: 0.0}, \
          orientation: {w: 1.0}}}"
```

**Via Python:**
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class NavigationGoalPublisher(Node):
    def __init__(self):
        super().__init__('nav_goal_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def send_goal(self, x, y, yaw=0.0):
        import math
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Convert yaw to quaternion
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.publisher.publish(goal)
        self.get_logger().info(f'Sent goal: ({x}, {y}, {yaw})')

def main():
    rclpy.init()
    node = NavigationGoalPublisher()
    node.send_goal(2.0, 1.0, 0.0)  # Navigate to (2, 1)
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Tuning for Your Robot

### Velocity Limits
Adjust in `controller_server` section:
```yaml
max_vel_x: 0.5      # Increase for faster robots
max_vel_theta: 1.0  # Adjust for turning speed
acc_lim_x: 2.5      # Acceleration limit
```

### Robot Footprint
Change robot size in costmaps:
```yaml
robot_radius: 0.3   # For circular footprint
# OR use footprint for complex shapes:
footprint: "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"
```

### Goal Tolerance
Adjust precision in `general_goal_checker`:
```yaml
xy_goal_tolerance: 0.15    # Position tolerance (meters)
yaw_goal_tolerance: 0.25   # Orientation tolerance (radians)
```

### Costmap Tuning
```yaml
inflation_radius: 0.55     # Safety margin around obstacles
cost_scaling_factor: 3.0   # Cost gradient steepness
```

## Troubleshooting

### Robot doesn't move
- **Check velocity commands:** `ros2 topic echo /cmd_vel`
- **Verify controller is running:** `ros2 node list | grep controller`
- **Check if goal is reachable:** Look at global plan in RViz

### Robot oscillates or shakes
- Reduce `controller_frequency` (default: 20 Hz)
- Increase `progress_checker.movement_time_allowance`
- Adjust DWB critics weights

### Robot gets stuck
- Check costmaps in RViz (add `Costmap` displays)
- Increase `inflation_radius` if too close to walls
- Verify sensor data (`ros2 topic echo /scan`)

### Path goes through obstacles
- Increase costmap resolution
- Adjust `obstacle_max_range` in sensor config
- Check TF transforms: `ros2 run tf2_tools view_frames`

### Recovery behaviors trigger constantly
- Increase `progress_checker.required_movement_radius`
- Check odometry drift
- Verify localization is working

## Topics

Key Nav2 topics:
```bash
# Input
/goal_pose              # Navigation goal
/initialpose            # Initial robot pose for localization
/scan                   # Laser scan data
/odom                   # Odometry

# Output
/cmd_vel                # Velocity commands to robot
/plan                   # Global path
/local_plan             # Local trajectory

# Visualization
/global_costmap/costmap
/local_costmap/costmap
/map                    # SLAM/localization map
```

## Related Book Chapters

- **Module 3.3**: Nav2 Path Planning
- **Module 3.2**: Isaac ROS Integration
- **Module 4**: Vision-Language-Action Systems (for semantic navigation)

## Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/index.html)
- [Behavior Trees](https://navigation.ros.org/behavior_trees/index.html)
- [Costmap Configuration](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)
