---
sidebar_position: 0
---

# Prerequisites: Setting Up Your ROS 2 Environment

Before diving into ROS 2 development, you need a properly configured environment. This chapter guides you through installing ROS 2 Humble on Ubuntu 22.04 and verifying your setup.

## System Requirements

**Recommended Configuration:**
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **RAM**: 8 GB minimum, 16 GB recommended
- **Storage**: 20 GB free space for ROS 2 and dependencies
- **CPU**: 64-bit x86 or ARM processor

**Alternative Platforms:**
- **Windows 10/11**: Use WSL2 (Windows Subsystem for Linux) with Ubuntu 22.04
- **macOS**: Docker container or virtual machine (native support is limited)

## Installing ROS 2 Humble

ROS 2 Humble Hawksbill is the LTS (Long-Term Support) release, supported until May 2027. Follow these steps:

### Step 1: Set Locale

```bash
# Ensure UTF-8 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Add ROS 2 Repository

```bash
# Add ROS 2 GPG key
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Packages

```bash
# Update package index
sudo apt update

# Install ROS 2 Humble Desktop (includes RViz, demos, tutorials)
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y
```

**Note**: The `ros-humble-desktop` package (~2 GB) includes visualization tools. For minimal installations (e.g., on embedded systems), use `ros-humble-ros-base` instead.

### Step 4: Source the Setup Script

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to your .bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verifying Installation

### Test 1: Check ROS 2 Version

```bash
ros2 --version
# Expected output: ros2 cli version X.X.X
```

### Test 2: Run Demo Talker-Listener

Open two terminals.

**Terminal 1 (Talker):**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2 (Listener):**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see the talker publishing messages and the listener receiving them:
```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [listener]: I heard: [Hello World: 1]
```

## Installing Additional Tools

### Colcon Build Tool

Colcon is the standard build tool for ROS 2 workspaces:

```bash
sudo apt install python3-colcon-common-extensions -y
```

### RViz2 (3D Visualization)

Included in `ros-humble-desktop`, but verify:

```bash
rviz2
```

A 3D visualization window should open.

### Gazebo Classic (Simulation)

```bash
sudo apt install ros-humble-gazebo-ros-pkgs -y
```

**Note**: Gazebo Ignition (newer version) is also available but requires additional configuration.

## Creating a ROS 2 Workspace

Workspaces organize your ROS 2 packages. Create one:

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace (empty for now)
colcon build

# Source the workspace overlay
source install/setup.bash
```

**Workspace Structure:**
```
ros2_ws/
├── build/       # Build artifacts
├── install/     # Installed packages
├── log/         # Build logs
└── src/         # Source code for packages
```

## Python Development Setup

ROS 2 supports both C++ and Python. For Python development:

```bash
# Install Python 3 dependencies
sudo apt install python3-pip python3-venv -y

# Install commonly used Python packages
pip3 install numpy matplotlib
```

**Best Practice**: Use virtual environments to isolate project dependencies, though ROS 2's system packages should be used as-is.

## Troubleshooting Common Issues

### Issue 1: "ros2: command not found"

**Solution**: Ensure you've sourced the setup script:
```bash
source /opt/ros/humble/setup.bash
```

### Issue 2: Colcon Build Fails with "Permission Denied"

**Solution**: Never use `sudo` with `colcon build`. Build as a regular user.

### Issue 3: RViz2 Fails to Launch (WSL2 on Windows)

**Solution**: Install VcXsrv or X410 for GUI support, and set:
```bash
export DISPLAY=:0
```

### Issue 4: Gazebo Crashes on Startup

**Solution**: Update graphics drivers. For Intel/AMD integrated graphics:
```bash
sudo apt install mesa-utils -y
```

## Next Steps

With ROS 2 installed and verified, you're ready to explore the architecture and build your first nodes. In the next chapter, we'll dive into ROS 2's core concepts: nodes, topics, services, and actions.

## References

- Open Robotics. (2024). "ROS 2 Humble Documentation." Retrieved from https://docs.ros.org/en/humble/
- Macenski, S., et al. (2022). "Robot Operating System 2: Design, architecture, and uses in the wild." *Science Robotics*, 7(66).
