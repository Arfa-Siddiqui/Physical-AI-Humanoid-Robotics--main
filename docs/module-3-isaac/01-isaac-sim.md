---
sidebar_position: 1
---

# Isaac Sim: GPU-Accelerated Robot Simulation

Isaac Sim is NVIDIA's next-generation robotics simulator built on Omniverse, offering photorealistic rendering, GPU-accelerated physics (PhysX 5), and synthetic data generation at scale. This chapter explores Isaac Sim's capabilities for humanoid robot development.

## Why Isaac Sim?

Compared to Gazebo or PyBullet, Isaac Sim provides:

- **RTX Ray Tracing**: Photorealistic lighting for vision algorithms
- **GPU Physics**: Simulate 1000+ robots in parallel on a single GPU
- **Synthetic Data**: Labeled images (bounding boxes, segmentation) for training
- **Digital Twins**: Import CAD models (USD format) with material properties
- **ROS 2 Integration**: Bidirectional communication with ROS 2 stacks

**Use Cases:**
- Training perception models (object detection, pose estimation)
- Reinforcement learning at scale (Isaac Gym integration)
- Warehouse automation (Nav2 + manipulation)

## Isaac Sim Interface Overview

### Main Window

- **Viewport**: 3D scene rendering
- **Stage**: Hierarchical tree of objects (USD prims)
- **Property**: Edit selected object properties
- **Console**: Python scripting and logs

### Toolbar

- **Play/Pause/Stop**: Control simulation
- **Camera**: Navigate viewport (WASD + mouse)
- **Transform**: Move, rotate, scale objects

## Creating a Humanoid Scene

### Step 1: Load a Base Environment

**File → New from Base Sample → Physics/Simple Room**

This loads a room with floor, walls, and default lighting.

### Step 2: Import a Robot (URDF or USD)

**Isaac Sim → Import Robot → Select URDF:**

Navigate to your `humanoid.urdf`. Isaac Sim converts it to USD and creates:
- **ArticulationRoot**: Root link with physics
- **ArticulationJoint**: Revolute/prismatic joints
- **VisualMesh**: Rendered geometry
- **CollisionMesh**: Physics collision shapes

**Adjust Placement:**

In Property panel:
- **Transform → Position**: `(0, 0, 0.5)`
- **Transform → Rotation**: `(0, 0, 0)`

### Step 3: Configure Physics

Select robot root → **Property → Physics → Articulation**:

- **Solver Position Iteration Count**: 32 (higher = more stable)
- **Enable Self Collision**: True
- **Fix Base Link**: False (for mobile robots)

**Joint Properties:**

Select individual joints → **Physics → Articulation Joint**:

- **Stiffness**: 10000 (position control strength)
- **Damping**: 1000 (reduce oscillation)
- **Armature**: 0.01 (accounts for motor inertia)

## Adding Sensors

### RGB Camera

1. **Create → Camera**
2. Position at robot's head
3. **Property → Camera**:
   - **Focal Length**: 24mm
   - **Horizontal Aperture**: 20.95mm
   - **Resolution**: 640×480

**Enable ROS 2 Publishing:**

1. **Isaac Sim → ROS2 → Camera Helper**
2. Select camera → **Publish Camera Info**: True
3. Topic name: `/robot/camera/image_raw`

Press Play. Camera images publish to ROS 2.

### Depth Camera (RGB-D)

Similar to RGB camera, but enable:

**Property → Camera → Depth:**
- **Rendering Mode**: Distance to Image Plane

Publishes depth images to `/robot/camera/depth`.

### LiDAR (Rotating Laser Scanner)

1. **Create → Isaac → Sensors → Rotating Lidar**
2. Select preset: **Velodyne VLP-16**
3. Position on robot torso
4. **Property → Lidar**:
   - **Horizontal Resolution**: 0.4° (900 samples/scan)
   - **Rotation Frequency**: 10 Hz
   - **Min/Max Range**: 0.1m to 100m

**Visualize Point Cloud:**

```bash
ros2 run rviz2 rviz2
```

Add PointCloud2 display, topic `/robot/lidar/point_cloud`.

### IMU

1. **Create → Isaac → Sensors → IMU Sensor**
2. Parent to robot base link
3. Publishes to `/robot/imu` (linear acceleration, angular velocity)

## Materials and Lighting for Realism

### PBR Materials

Apply Physically Based Rendering materials:

1. Select mesh → **Property → Material**
2. **Shader**: `OmniPBR`
3. **Albedo**: Base color texture
4. **Roughness**: 0.5 (semi-glossy)
5. **Metallic**: 0.8 (for metal parts)

**Example**: Robot frame → Metallic=0.9, Roughness=0.3 (shiny metal)

### HDR Lighting

For realistic lighting:

1. **Create → Light → Dome Light**
2. **Texture**: Load HDR environment map (e.g., `warehouse.hdr`)
3. **Intensity**: 1000

**Dynamic Lighting:**

Add **Distant Light** (sun):
- **Intensity**: 50000
- **Angle**: (45°, 0°) (morning sun)

## Synthetic Data Generation

### Replicator: Randomization at Scale

Isaac Sim's Replicator generates diverse training data.

**Example: Randomize Object Placement**

Python script in Isaac Sim:

```python
import omni.replicator.core as rep

# Define randomizer
def randomize_scene():
    with rep.new_layer():
        # Randomize lighting
        light = rep.create.light(light_type="Distant")
        rep.modify.attribute(light, "inputs:intensity", rep.distribution.uniform(10000, 100000))

        # Randomize object positions
        objects = rep.get.prims(path_pattern="/World/Objects/*")
        with objects:
            rep.modify.pose(
                position=rep.distribution.uniform((-5, 0, -5), (5, 0, 5))
            )

# Trigger randomization every 10 frames
rep.randomizer.register(randomize_scene)
rep.orchestrator.run()
```

Press Play. Scene randomizes every 10 frames.

### Bounding Box Annotations

Enable object detection labels:

1. **Isaac Sim → Replicator → Semantic Schema Editor**
2. Assign classes: "robot", "table", "chair"
3. **Isaac Sim → Replicator → Writer → Basic Writer**
4. Output path: `/tmp/isaac_data`
5. Enable **Bounding Box 2D Tight** annotations

Data saved as JSON with image paths and bounding boxes.

## Controlling the Robot

### Joint Position Control

Python API:

```python
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation

# Initialize world
world = World()
world.reset()

# Get robot articulation
robot = world.scene.get_object("my_humanoid")

# Set joint positions (radians)
joint_positions = {
    "left_shoulder_joint": 1.0,
    "right_shoulder_joint": -1.0,
    "left_elbow_joint": 0.5
}
robot.set_joint_positions(joint_positions)

# Simulate one step
world.step(render=True)
```

### Velocity Control via ROS 2

Subscribe to `/cmd_vel`:

1. **Isaac Sim → ROS2 → Differential Controller**
2. Select robot base → Configure wheel joints
3. Subscribe to `/cmd_vel`

Publish from terminal:

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
```

Robot moves in Isaac Sim.

## Performance Optimization

### Reduce Rendering Load

**Settings → Rendering:**
- **Anti-Aliasing**: TAA (fast) instead of DLSS
- **Raytracing Samples**: 1 (real-time) vs 16 (offline quality)
- **Reflections/Shadows**: Disable if not needed

### Physics Substeps

**Edit → Preferences → Physics:**
- **Substeps**: 2 (balance speed/accuracy)
- **GPU Dynamics**: Enabled (requires RTX GPU)

## Conclusion

Isaac Sim combines photorealistic rendering with GPU-accelerated physics, enabling simulation at scales impossible with traditional tools. Whether training perception models or testing navigation stacks, Isaac Sim provides the fidelity and performance modern robotics demands. In the next chapter, we'll integrate Isaac ROS for GPU-accelerated perception nodes.

## References

- NVIDIA. (2024). "Isaac Sim Documentation." Retrieved from https://docs.omniverse.nvidia.com/isaacsim/latest/
- Makoviychuk, V., et al. (2021). "Isaac Gym: High Performance GPU-Based Physics Simulation for Robot Learning." *arXiv preprint arXiv:2108.10470*.
