---
sidebar_position: 2
---

# Gazebo: Simulating Humanoid Robots

Gazebo is the standard simulator for ROS, providing physics simulation, sensor emulation, and plugin extensibility. This chapter guides you through spawning a humanoid robot, configuring physics, and integrating sensors in Gazebo.

## Launching Gazebo with ROS 2

### Empty World

Start Gazebo with a minimal environment:

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

This opens Gazebo with:
- Ground plane
- Default lighting (sun)
- Empty scene

###Adding a Robot via Launch File

Create `spawn_robot.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'humanoid.urdf')

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_humanoid',
                '-file', urdf_file,
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['cat ', urdf_file])}]
        )
    ])
```

**Run:**

```bash
ros2 launch my_robot_description spawn_robot.launch.py
```

## URDF Gazebo Extensions

Standard URDF defines visual and collision geometry. Gazebo requires additional tags for physics and sensors.

### Material Properties

Specify friction and bounce:

```xml
<gazebo reference="foot_link">
  <mu1>0.9</mu1>    <!-- Friction coefficient (direction 1) -->
  <mu2>0.9</mu2>    <!-- Friction coefficient (direction 2) -->
  <kp>1000000.0</kp> <!-- Contact stiffness -->
  <kd>1.0</kd>       <!-- Contact damping -->
  <material>Gazebo/Grey</material>
</gazebo>
```

### Self-Collision

Enable collision checking between robot links:

```xml
<gazebo>
  <self_collide>true</self_collide>
</gazebo>
```

## Gazebo Plugins for Actuation

Plugins enable control interfaces between ROS 2 and Gazebo.

### Differential Drive (Mobile Base)

For wheeled humanoids:

```xml
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/robot</namespace>
    </ros>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
  </plugin>
</gazebo>
```

Subscribes to `/robot/cmd_vel` (Twist messages) and moves the robot.

### Joint State Publisher

Publish joint positions and velocities:

```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
      <namespace>/robot</namespace>
      <argument>~/out:=joint_states</argument>
    </ros>
    <update_rate>50</update_rate>
  </plugin>
</gazebo>
```

Publishes to `/robot/joint_states` (sensor_msgs/JointState).

### Joint Position Controller

Control individual joints:

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find my_robot_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

**Controller Configuration** (`controllers.yaml`):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controller:
      type: position_controllers/JointGroupPositionController

position_controller:
  ros__parameters:
    joints:
      - left_shoulder_joint
      - right_shoulder_joint
      - left_elbow_joint
      - right_elbow_joint
```

**Command Joint:**

```bash
ros2 topic pub /position_controller/commands std_msgs/Float64MultiArray "data: [1.0, -1.0, 0.5, -0.5]"
```

## Sensor Simulation

### Camera Plugin

Add to `<gazebo reference="camera_link">`:

```xml
<sensor type="camera" name="head_camera">
  <update_rate>30.0</update_rate>
  <visualize>true</visualize>
  <camera>
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.02</near>
      <far>300</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
    </ros>
    <camera_name>head_camera</camera_name>
    <frame_name>camera_link</frame_name>
    <hack_baseline>0.07</hack_baseline>
  </plugin>
</sensor>
```

**View Image:**

```bash
ros2 run rqt_image_view rqt_image_view /robot/head_camera/image_raw
```

### LiDAR Plugin

```xml
<sensor type="ray" name="lidar">
  <pose>0 0 0.1 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <argument>~/out:=/robot/scan</argument>
    </ros>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>
```

**Visualize in RViz:**

```bash
ros2 run rviz2 rviz2
```

Add LaserScan display, set topic to `/robot/scan`.

### IMU Plugin

```xml
<sensor name="imu" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/robot</namespace>
    </ros>
    <frame_name>imu_link</frame_name>
  </plugin>
</sensor>
```

Publishes to `/robot/imu` (sensor_msgs/Imu).

## World Files: Defining Environments

Create `office.world`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="office">
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add obstacles -->
    <model name="box1">
      <pose>2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 1.0</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 1.0</size></box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**Launch with World:**

```python
ExecuteProcess(
    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 'office.world'],
    output='screen'
)
```

## Debugging Tips

### Check Available Topics

```bash
ros2 topic list
```

Should show `/robot/joint_states`, `/robot/cmd_vel`, etc.

### Monitor Transform Tree

```bash
ros2 run tf2_tools view_frames
```

Generates `frames.pdf` showing TF relationships.

### Gazebo GUI Tips

- **Pause/Play**: Control simulation time
- **Insert Tab**: Add models dynamically
- **View→Transparent**: See collisions and CoM

## Performance Optimization

### Reduce Rendering Quality

```bash
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
export OGRE_RTShader_Mode=0  # Disable shaders
gazebo --verbose
```

### Increase Physics Timestep (Less Accurate, Faster)

In `.world` file:

```xml
<physics type="ode">
  <max_step_size>0.002</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

## Conclusion

Gazebo enables full-stack robot testing in simulation: spawn robots, configure physics, add sensors, and integrate with ROS 2 controllers. Before deploying to hardware, validate behaviors in Gazebo to catch issues early. In the next chapter, we'll explore Unity for high-fidelity visualization and synthetic data generation.

## References

- Koenig, N., & Howard, A. (2004). "Design and use paradigms for Gazebo, an open-source multi-robot simulator." *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 2149-2154.
- Open Robotics. (2024). "Gazebo ROS 2 Integration." Retrieved from https://github.com/ros-simulation/gazebo_ros_pkgs
