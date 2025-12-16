---
sidebar_position: 3
---

# URDF: Modeling Humanoid Robots

A robot description defines its physical structure—links, joints, sensors, and visual/collision geometry. The **Unified Robot Description Format (URDF)** is ROS's XML-based standard for robot modeling. This chapter teaches you to create URDF models for humanoid robots, enabling simulation and motion planning.

## URDF Fundamentals

URDF models consist of two primary elements:

### Links: Rigid Bodies

A **link** represents a rigid body with:
- **Visual geometry**: What the link looks like (meshes, primitive shapes)
- **Collision geometry**: Simplified shapes for physics simulation
- **Inertial properties**: Mass, center of mass, inertia tensor

Example link (simple box):

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="10.0"/>
    <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

### Joints: Connections Between Links

A **joint** connects two links and defines their relative motion. Types:

| Joint Type | Motion | Example |
|------------|--------|---------|
| **Fixed** | No motion | Sensor mounts |
| **Revolute** | Rotation (limited angle) | Elbows, knees |
| **Continuous** | Rotation (unlimited) | Wheels |
| **Prismatic** | Linear translation | Telescoping arms |
| **Planar** | 2D translation | Rarely used |
| **Floating** | 6 DOF (position + orientation) | Base of mobile robot |

Example revolute joint (elbow):

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="0.5" effort="50" velocity="2.0"/>
</joint>
```

**Breakdown:**
- **`parent/child`**: Links being connected
- **`origin`**: Position (`xyz`) and orientation (`rpy` = roll-pitch-yaw) of child relative to parent
- **`axis`**: Rotation axis (here, y-axis)
- **`limit`**: Joint limits (radians), max torque (Nm), max velocity (rad/s)

## Building a Humanoid: Hierarchical Structure

A humanoid robot has a **kinematic tree** rooted at the torso:

```
                    torso (base_link)
                      |
         ┌────────────┼────────────┐
         |            |            |
       head      left_arm      right_arm
                      |
                 ┌────┴────┐
            left_leg    right_leg
```

Each limb is a chain of links connected by joints.

## Minimal Humanoid URDF

Let's build a simplified humanoid with torso, head, and two arms:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link (Torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.6" ixy="0" ixz="0" iyy="0.6" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Left Arm (Upper) -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.03"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.15 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="30" velocity="2.0"/>
  </joint>

  <!-- Right Arm (mirror left arm structure) -->
  <!-- ... (omitted for brevity) -->

</robot>
```

## Visualizing URDF in RViz

**Launch RViz with Robot State Publisher:**

1. Create a launch file `view_robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os

def generate_launch_description():
    urdf_path = os.path.join(
        os.path.dirname(__file__),
        'simple_humanoid.urdf'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', urdf_path]),
                    value_type=str
                )
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                os.path.dirname(__file__),
                'config.rviz'
            )]
        )
    ])
```

2. Run:

```bash
ros2 launch my_robot_description view_robot.launch.py
```

RViz opens, showing the robot. The Joint State Publisher GUI lets you move joints interactively.

## Xacro: Macros for Reusable URDF

URDF is verbose. **Xacro** adds macros and variables for cleaner definitions.

Example: Define arm as a macro and instantiate twice (left/right):

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">

  <xacro:macro name="arm" params="prefix reflect">
    <link name="${prefix}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="0.25" radius="0.03"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${prefix}_shoulder_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_upper_arm"/>
      <origin xyz="0 ${reflect * 0.15} 0.2" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="-3.14" upper="3.14" effort="30" velocity="2.0"/>
    </joint>
  </xacro:macro>

  <link name="base_link">
    <!-- ... -->
  </link>

  <xacro:arm prefix="left" reflect="1"/>
  <xacro:arm prefix="right" reflect="-1"/>

</robot>
```

**Convert Xacro to URDF:**

```bash
xacro my_robot.urdf.xacro > my_robot.urdf
```

## Adding Sensors: Cameras and LiDAR

Sensors are links with **Gazebo plugins**.

### Camera

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </collision>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <argument>~/image_raw:=camera/image_raw</argument>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <pose>0 0 0 0 0 0</pose>
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
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <argument>~/out:=scan</argument>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Mesh Models for Realistic Appearance

Replace primitive shapes with 3D meshes:

```xml
<visual>
  <geometry>
    <mesh filename="package://my_robot_description/meshes/torso.stl" scale="1 1 1"/>
  </geometry>
</visual>
```

**Supported Formats**: STL, DAE (COLLADA), OBJ

**Best Practices**:
- Use simplified meshes for collision (convex hulls)
- Keep visual meshes under 50k polygons for performance

## Testing in Gazebo

Launch robot in Gazebo:

```bash
ros2 launch my_robot_description gazebo.launch.py
```

Gazebo will:
- Load URDF physics (masses, inertias, joint limits)
- Simulate sensors (publish camera images, LiDAR scans)
- Accept `/cmd_vel` for control

## Conclusion

URDF modeling is essential for simulation and visualization. You've learned to define links, joints, sensors, and leverage Xacro for modularity. In the next module, we'll use these models in Gazebo for physics simulation and testing before deploying to real hardware.

## References

- Coleman, D., et al. (2014). "Reducing the Barrier to Entry of Complex Robotic Software: a MoveIt! Case Study." *Journal of Software Engineering for Robotics*, 5(1).
- Open Robotics. (2024). "URDF Tutorials." Retrieved from https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
