---
sidebar_position: 0
---

# Prerequisites: Setting Up Simulation Environments

Simulation is indispensable for robotics development—it's faster, safer, and cheaper than real-world testing. This chapter prepares your environment for physics simulation with Gazebo and visualization with Unity.

## Why Simulation?

Before deploying to physical hardware, simulation enables:
- **Rapid iteration**: Test changes in seconds instead of hours
- **Safety**: Robots can fall, collide, or fail without damage
- **Scalability**: Run thousands of trials in parallel (reinforcement learning)
- **Reproducibility**: Exact conditions for debugging and benchmarking

## Gazebo: Physics Simulation

**Gazebo** (formerly Gazebo Classic, now Gazebo Sim/Ignition) is the de facto simulator for ROS. It provides:
- Realistic physics (gravity, friction, contact dynamics)
- Sensor simulation (cameras, LiDAR, IMU)
- Plugin system for custom behaviors

### Installing Gazebo Classic (for ROS 2 Humble)

Gazebo Classic integrates seamlessly with ROS 2 Humble:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs -y
```

**Verify Installation:**

```bash
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x
```

**Launch Empty World:**

```bash
gazebo
```

A window with a ground plane should appear.

### Installing Gazebo Ignition (New Generation)

Gazebo Ignition (now called **Gazebo Sim**) is the next-generation simulator with improved graphics and modularity.

```bash
sudo apt install ros-humble-ros-gz -y
```

**Note**: ROS 2 Humble officially supports Gazebo Fortress. For newer versions (Garden, Harmonic), manual setup is required.

**Launch Fortress:**

```bash
ign gazebo
```

## GPU Acceleration

Physics and rendering benefit significantly from GPU acceleration.

### Check GPU

```bash
lspci | grep -i vga
```

### Install NVIDIA Drivers (for NVIDIA GPUs)

```bash
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
sudo reboot
```

### Verify CUDA (Optional, for GPU-accelerated physics)

```bash
nvidia-smi
```

**Note**: Gazebo uses CPU physics by default. GPU physics (via NVIDIA PhysX) requires Isaac Sim (covered in Module 3).

## Unity: High-Fidelity Visualization

While Gazebo handles physics, **Unity** excels at photorealistic rendering for computer vision and synthetic data generation.

### Installing Unity Hub

1. Download Unity Hub from [unity.com](https://unity.com/download)
2. Install Unity Editor LTS (2021.3 or newer)

### Unity Robotics Hub

Unity Technologies provides ROS integration:

```bash
# Clone Unity Robotics Hub
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
```

**Install ROS TCP Connector:**

```bash
sudo apt install ros-humble-ros-tcp-endpoint -y
```

This enables bidirectional communication between ROS 2 and Unity.

## Testing the Setup

### Test 1: Spawn a Robot in Gazebo

```bash
# Launch TurtleBot3 in Gazebo (demo robot)
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

You should see a TurtleBot3 in an empty environment.

**Control the Robot:**

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Use WASD keys to drive.

### Test 2: Visualize in RViz

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

In another terminal:

```bash
ros2 run rviz2 rviz2
```

Add displays:
- **RobotModel**: Shows URDF
- **LaserScan**: Visualizes LiDAR data from `/scan`

## Docker Alternative (Cross-Platform)

For Windows/macOS, use Docker:

### Dockerfile for ROS 2 + Gazebo

```dockerfile
FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo \
    && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
```

**Build and Run:**

```bash
docker build -t ros2-gazebo .
docker run -it --rm --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix ros2-gazebo
```

**Note**: Requires X server (VcXsrv on Windows, XQuartz on macOS).

## Performance Tuning

### Reduce Graphics Quality

If Gazebo runs slowly:

```bash
# Disable shadows
export OGRE_RTShader_Mode=0
gazebo --verbose
```

### Real-Time Factor

Gazebo's real-time factor (RTF) indicates simulation speed. RTF = 1.0 means real-time. RTF < 1.0 means slower than real-time (complex physics).

**Check RTF:**

In Gazebo GUI, bottom-left shows: "Real Time Factor: 0.85"

## Common Issues

### Issue 1: Gazebo Crashes on Startup

**Solution**: Update Mesa drivers:

```bash
sudo apt install --reinstall libgl1-mesa-glx libgl1-mesa-dri -y
```

### Issue 2: No Camera Image in RViz

**Solution**: Ensure Gazebo camera plugin is loaded. Check topic:

```bash
ros2 topic list | grep image
```

### Issue 3: Unity ROS Connection Fails

**Solution**: Verify ROS TCP Endpoint is running:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```

## Next Steps

With Gazebo and Unity installed, you're ready to simulate physics, test controllers, and generate synthetic data. In the next chapter, we'll explore physics fundamentals—how simulators model the real world and the trade-offs involved.

## References

- Koenig, N., & Howard, A. (2004). "Design and use paradigms for Gazebo, an open-source multi-robot simulator." *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 2149-2154.
- Open Robotics. (2024). "Gazebo Tutorials." Retrieved from https://gazebosim.org/docs
