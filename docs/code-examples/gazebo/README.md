# Gazebo Simulation Examples

Gazebo launch files and URDF models for simulating humanoid robots.

## Prerequisites

- ROS 2 Humble
- Gazebo Classic 11 or Gazebo Fortress
- `gazebo_ros_pkgs` installed:
  ```bash
  sudo apt install ros-humble-gazebo-ros-pkgs
  ```

## Files

### 1. `humanoid_world.launch.py`
Complete launch file that starts Gazebo with a humanoid robot

**Features:**
- Launches Gazebo server and client
- Spawns humanoid robot at origin
- Starts robot state publisher
- Includes joint state publisher GUI for manual control

**Usage:**
```bash
ros2 launch <package_name> humanoid_world.launch.py
```

**With custom world:**
```bash
ros2 launch <package_name> humanoid_world.launch.py world_file:=/path/to/world.world
```

### 2. `simple_humanoid.urdf`
Simplified humanoid robot URDF with:
- Torso (base_link)
- Head with neck joint
- Left and right arms with shoulder joints
- Left and right legs with hip joints
- Gazebo plugins for joint state publishing and control

**Visualize in RViz:**
```bash
# Terminal 1: Publish robot description
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat simple_humanoid.urdf)"

# Terminal 2: Launch RViz
ros2 run rviz2 rviz2
```

**In RViz:**
1. Set Fixed Frame to `base_link`
2. Add `RobotModel` display
3. Set `Description Topic` to `/robot_description`

## Integration Steps

### Option 1: Standalone Launch

1. Copy files to your workspace
2. Run launch file directly:
   ```bash
   ros2 launch humanoid_world.launch.py
   ```

### Option 2: Create ROS 2 Package

1. Create package:
   ```bash
   ros2 pkg create --build-type ament_cmake humanoid_gazebo \
     --dependencies gazebo_ros robot_state_publisher
   ```

2. Directory structure:
   ```
   humanoid_gazebo/
   ├── launch/
   │   └── humanoid_world.launch.py
   ├── urdf/
   │   └── simple_humanoid.urdf
   └── CMakeLists.txt
   ```

3. Update `CMakeLists.txt`:
   ```cmake
   install(DIRECTORY launch urdf
     DESTINATION share/${PROJECT_NAME}
   )
   ```

4. Build:
   ```bash
   colcon build --packages-select humanoid_gazebo
   source install/setup.bash
   ```

5. Launch:
   ```bash
   ros2 launch humanoid_gazebo humanoid_world.launch.py
   ```

## Controlling the Robot

### Using Joint State Publisher GUI
The launch file includes a GUI for manual joint control:
- Sliders appear for each joint
- Move sliders to control joint angles in real-time

### Using ROS 2 Topics
Publish joint commands programmatically:

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Create trajectory message
traj = JointTrajectory()
traj.joint_names = ['left_shoulder', 'right_shoulder', 'left_hip', 'right_hip']

point = JointTrajectoryPoint()
point.positions = [0.5, 0.5, -0.3, -0.3]
point.time_from_start.sec = 1

traj.points = [point]

# Publish to /joint_trajectory topic
publisher.publish(traj)
```

## Troubleshooting

**Gazebo doesn't start:**
- Check Gazebo is installed: `gazebo --version`
- Source ROS 2 setup: `source /opt/ros/humble/setup.bash`

**Robot doesn't appear:**
- Check URDF is valid: `check_urdf simple_humanoid.urdf`
- Verify spawn position is above ground (z > 0)

**Joints don't move:**
- Ensure Gazebo plugins are loaded
- Check joint limits in URDF
- Verify topics with: `ros2 topic list`

## Related Book Chapters

- **Module 2.1**: Physics Fundamentals
- **Module 2.2**: Gazebo Simulation
- **Module 2.3**: Unity Visualization
