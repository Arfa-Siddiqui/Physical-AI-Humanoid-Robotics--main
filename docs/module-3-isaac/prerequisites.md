---
sidebar_position: 0
---

# Prerequisites: NVIDIA Isaac Platform Setup

NVIDIA Isaac is a platform for AI-powered robotics, combining GPU-accelerated simulation (Isaac Sim), ROS integration (Isaac ROS), and pre-trained models for perception. This chapter prepares your system for Isaac development.

## System Requirements

**Minimum:**
- **GPU**: NVIDIA RTX 2060 or better (6 GB VRAM)
- **CPU**: Intel i7 or AMD Ryzen 7
- **RAM**: 16 GB
- **Storage**: 50 GB free space
- **OS**: Ubuntu 20.04 or 22.04

**Recommended:**
- **GPU**: RTX 3080 or better (10+ GB VRAM) for real-time ray tracing
- **RAM**: 32 GB for large scenes
- **Storage**: NVMe SSD for fast asset loading

**Note**: Isaac Sim requires an NVIDIA GPU. AMD/Intel GPUs are not supported.

## Installing NVIDIA Drivers

### Check Current Driver

```bash
nvidia-smi
```

You need driver version **≥ 525** for Isaac Sim 2023+.

### Update Driver

```bash
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
sudo reboot
```

**Verify:**

```bash
nvidia-smi
# Check "Driver Version" in output
```

## Isaac Sim Installation

Isaac Sim is built on NVIDIA Omniverse, a platform for 3D collaboration and simulation.

### Option 1: Omniverse Launcher (Recommended)

1. Download **Omniverse Launcher**: https://www.nvidia.com/en-us/omniverse/download/
2. Install launcher:

```bash
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

3. Open Launcher → **Exchange** tab
4. Install **Isaac Sim** (latest version, ~20 GB download)

### Option 2: Docker Container

For headless servers or consistent environments:

```bash
docker pull nvcr.io/nvidia/isaac-sim:2023.1.0
```

**Run with Display:**

```bash
docker run --gpus all -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  nvcr.io/nvidia/isaac-sim:2023.1.0
```

## Launching Isaac Sim

**From Omniverse Launcher:**

Click **Isaac Sim → Launch**

**From Command Line:**

```bash
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
```

Isaac Sim window opens. Initial load takes 1-2 minutes (compiling shaders).

## Isaac ROS Installation

Isaac ROS provides GPU-accelerated ROS 2 nodes for perception and navigation.

### Install Isaac ROS (Docker Method)

**Prerequisites:**

```bash
# Install Docker
curl https://get.docker.com | sh
sudo usermod -aG docker $USER
newgrp docker

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

**Pull Isaac ROS Docker:**

```bash
docker pull nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_692e1c3bedd77b4a8e2e9cdff0b0e4ba
```

**Run Container:**

```bash
docker run -it --gpus all --network host \
  nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_692e1c3bedd77b4a8e2e9cdff0b0e4ba
```

### Native Installation (Jetson/x86)

For NVIDIA Jetson or native Linux:

```bash
cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Verifying Installation

### Test 1: Isaac Sim Scene Load

In Isaac Sim:
1. **File → Open**
2. Navigate to `omniverse://localhost/NVIDIA/Assets/Isaac/2023.1/Isaac/Samples/ROS2/Scenario/simple_room.usd`
3. Press **Play** (bottom-left)

Scene loads with a simple robot in a room.

### Test 2: ROS 2 Bridge

In Isaac Sim:
1. **Window → Extensions**
2. Search "ROS2 Bridge" → Enable
3. Play scene
4. In terminal:

```bash
ros2 topic list
```

Should show Isaac Sim topics (e.g., `/tf`, `/joint_states`).

### Test 3: Isaac ROS AprilTag Detection

```bash
# Launch AprilTag detector
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

If successful, detector node starts and subscribes to `/image`.

## GPU Memory Management

Isaac Sim is memory-intensive. Monitor usage:

```bash
watch -n 1 nvidia-smi
```

**Reduce Memory:**
- Lower render resolution (Isaac Sim settings)
- Disable ray tracing
- Close other GPU applications

## Troubleshooting

### Issue 1: "CUDA Error: Out of Memory"

**Solution**: Reduce scene complexity or upgrade GPU. For RTX 2060, avoid complex scenes with >100k polygons.

### Issue 2: Isaac Sim Crashes on Launch

**Solution**: Update NVIDIA driver to latest stable version.

### Issue 3: ROS 2 Topics Not Visible

**Solution**: Ensure ROS 2 Bridge extension is enabled in Isaac Sim and domain ID matches:

```bash
export ROS_DOMAIN_ID=0
```

## Next Steps

With Isaac Sim and Isaac ROS installed, you're ready for GPU-accelerated robotics development. The next chapter explores Isaac Sim's capabilities: physics simulation, sensor generation, and synthetic data creation.

## References

- NVIDIA. (2024). "Isaac Sim Documentation." Retrieved from https://docs.omniverse.nvidia.com/isaacsim/latest/
- NVIDIA. (2024). "Isaac ROS Documentation." Retrieved from https://nvidia-isaac-ros.github.io/
