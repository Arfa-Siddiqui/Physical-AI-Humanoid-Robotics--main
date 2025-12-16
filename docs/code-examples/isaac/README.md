# NVIDIA Isaac Sim Examples

Python scripts for NVIDIA Isaac Sim with humanoid robots.

## Prerequisites

### Required Software
- **NVIDIA Isaac Sim 2023.1.0 or later**
  - Download: https://developer.nvidia.com/isaac-sim
  - Requires NVIDIA GPU (RTX series recommended)
- **ROS 2 Humble** (for ROS bridge examples)
- **Ubuntu 20.04/22.04**

### Installation
```bash
# Install Isaac Sim (follow official docs)
# After installation, Isaac Sim is typically at:
# ~/.local/share/ov/pkg/isaac_sim-2023.1.0/

# Add Isaac Sim Python to path
export ISAAC_SIM_PATH=~/.local/share/ov/pkg/isaac_sim-2023.1.0
export PYTHONPATH=$ISAAC_SIM_PATH/exts/omni.isaac.kit/omni/isaac/kit:$PYTHONPATH
```

## Files

### 1. `isaac_sim_setup.py`
Basic Isaac Sim setup with humanoid robot

**Features:**
- Creates scene with ground plane
- Loads built-in humanoid robot from Nucleus
- Initializes physics simulation
- Basic simulation loop

**Run:**
```bash
cd $ISAAC_SIM_PATH
./python.sh /path/to/isaac_sim_setup.py
```

**Expected Output:**
- Isaac Sim GUI opens
- Humanoid robot spawned 1m above ground
- Physics simulation runs for 1000 steps
- Robot position logged every 100 steps

### 2. `isaac_ros_bridge.py`
Isaac Sim with ROS 2 integration

**Features:**
- Publishes joint states to ROS 2
- Camera sensor with image publishing
- TF tree broadcasting
- Real-time ROS 2 communication

**Run:**
```bash
# Terminal 1: Source ROS 2
source /opt/ros/humble/setup.bash

# Terminal 2: Run Isaac Sim with ROS bridge
cd $ISAAC_SIM_PATH
source /opt/ros/humble/setup.bash  # Important: source before running
./python.sh /path/to/isaac_ros_bridge.py
```

**Verify ROS 2 topics:**
```bash
# In another terminal
ros2 topic list
# Expected topics:
# /joint_states
# /tf
# /camera/image_raw

# Echo joint states
ros2 topic echo /joint_states

# View camera feed
ros2 run rqt_image_view rqt_image_view
```

## Common Tasks

### Loading Custom URDF
```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Import URDF to USD first
from omni.isaac.urdf import _urdf

# Convert URDF to USD
urdf_interface = _urdf.acquire_urdf_interface()
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False

result = urdf_interface.parse_urdf(
    urdf_path="/path/to/robot.urdf",
    import_config=import_config
)

# Then load USD
add_reference_to_stage(
    usd_path="/path/to/converted/robot.usd",
    prim_path="/World/CustomRobot"
)
```

### Controlling Joints
```python
# Get robot
robot = world.scene.get_object("humanoid_robot")

# Set joint positions
joint_positions = [0.1, 0.2, 0.0, -0.1, ...]  # radians
robot.set_joint_positions(joint_positions)

# Or set joint velocities
joint_velocities = [0.5, 0.0, 0.0, ...]  # rad/s
robot.set_joint_velocities(joint_velocities)

# Apply joint efforts (torques)
joint_efforts = [10.0, 5.0, 3.0, ...]  # N⋅m
robot.set_joint_efforts(joint_efforts)
```

### Adding Sensors
```python
from omni.isaac.sensor import Camera, ContactSensor, IMUSensor

# Camera
camera = Camera(
    prim_path="/World/Camera",
    position=[2.0, 0, 1.5],
    resolution=(1280, 720),
    frequency=30
)

# IMU
imu = IMUSensor(
    prim_path="/World/Humanoid/imu",
    position=[0, 0, 0.5]
)

# Get sensor data
camera_data = camera.get_current_frame()
imu_data = imu.get_current_frame()
```

## Isaac Sim Concepts

### Prims
- Everything in Isaac Sim is a "prim" (primitive)
- Hierarchical structure like filesystem: `/World/Humanoid/LeftArm`
- Use `create_prim()` to add objects

### Physics
- PhysX physics engine (real-time, GPU-accelerated)
- Materials define friction, restitution
- Collision shapes separate from visual geometry

### Nucleus
- NVIDIA's asset repository
- Access via `get_assets_root_path()`
- Contains robots, environments, props

## Troubleshooting

### "Could not find assets"
```bash
# Check Nucleus connection
# Go to: Window → Nucleus in Isaac Sim
# Connect to localhost or NVIDIA's cloud Nucleus
```

### ROS 2 topics not appearing
```bash
# Ensure ROS 2 sourced before launching Isaac Sim
source /opt/ros/humble/setup.bash
cd $ISAAC_SIM_PATH
./python.sh script.py

# Check ROS 2 bridge is loaded
# In Isaac Sim: Window → Extensions → Search "ROS2 Bridge"
```

### Low frame rate
- Reduce resolution: Set `renderer: "RayTracedLighting"` to `"PathTracing"`
- Disable real-time mode
- Use headless mode for batch processing

### GPU memory errors
- Reduce scene complexity
- Lower texture resolution
- Close other GPU applications

## Performance Tips

1. **Use GPU for physics:**
   ```python
   world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0)
   ```

2. **Headless mode for training:**
   ```python
   simulation_app = SimulationApp({"headless": True})
   ```

3. **Async rendering:**
   ```python
   world.step(render=False)  # Physics only
   # Render every N steps
   if step % 10 == 0:
       world.render()
   ```

## Related Book Chapters

- **Module 3.1**: Isaac Sim Fundamentals
- **Module 3.2**: Isaac ROS Integration
- **Module 3.3**: Nav2 with Isaac Sim
- **Module 4**: Vision-Language-Action Systems

## Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/)
- [Isaac Sim Python API](https://docs.omniverse.nvidia.com/py/isaacsim/)
- [ROS 2 Bridge Guide](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html)
- [Isaac Gym](https://developer.nvidia.com/isaac-gym) - Reinforcement learning
