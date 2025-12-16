# ROS 2 Code Examples

These examples demonstrate core ROS 2 concepts covered in Module 1.

## Prerequisites

- ROS 2 Humble installed
- Python 3.8+
- Basic ROS 2 workspace setup

## Files

### 1. `simple_publisher.py`
Basic publisher node that publishes simulated sensor data to `/robot/sensor_data`

**Run:**
```bash
python3 simple_publisher.py
```

### 2. `simple_subscriber.py`
Subscriber node that receives sensor data and computes statistics

**Run:**
```bash
python3 simple_subscriber.py
```

**Test together:**
```bash
# Terminal 1
python3 simple_publisher.py

# Terminal 2
python3 simple_subscriber.py
```

### 3. `joint_controller.py`
PD controller for humanoid robot joints. Subscribes to desired joint positions and publishes velocity commands.

**Topics:**
- Subscribes: `/joint_states`, `/desired_joint_positions`
- Publishes: `/joint_velocity_commands`

**Run:**
```bash
python3 joint_controller.py
```

### 4. `simple_service_server.py`
Service server that computes distance traveled by robot

**Run:**
```bash
python3 simple_service_server.py
```

**Call service:**
```bash
ros2 service call /compute_distance std_srvs/srv/Trigger
```

## Integration with ROS 2 Package

To use these in a ROS 2 package:

1. Create a package:
```bash
ros2 pkg create --build-type ament_python humanoid_examples
```

2. Copy Python files to `humanoid_examples/humanoid_examples/`

3. Update `setup.py`:
```python
entry_points={
    'console_scripts': [
        'publisher = humanoid_examples.simple_publisher:main',
        'subscriber = humanoid_examples.simple_subscriber:main',
        'controller = humanoid_examples.joint_controller:main',
        'service = humanoid_examples.simple_service_server:main',
    ],
},
```

4. Build and source:
```bash
colcon build --packages-select humanoid_examples
source install/setup.bash
```

5. Run:
```bash
ros2 run humanoid_examples publisher
```

## Related Book Chapters

- **Module 1.2**: ROS 2 Architecture
- **Module 1.3**: Python Controllers
- **Module 1.4**: URDF for Humanoids
