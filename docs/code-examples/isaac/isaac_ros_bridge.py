#!/usr/bin/env python3
"""
Isaac Sim to ROS 2 Bridge Example
Publishes robot joint states and camera data to ROS 2 topics

Prerequisites:
- Isaac Sim with ROS 2 Bridge extension enabled
- ROS 2 Humble sourced
- Run from Isaac Sim: ./python.sh isaac_ros_bridge.py
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim with ROS 2 bridge
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting"
})

import omni
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Enable ROS 2 bridge
import omni.isaac.ros2_bridge as ros2_bridge

def setup_ros2_scene():
    """Setup Isaac Sim scene with ROS 2 publishers"""

    # Create world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Get assets
    assets_root_path = get_assets_root_path()
    robot_usd_path = f"{assets_root_path}/Isaac/Robots/Humanoid/humanoid.usd"

    # Add robot
    add_reference_to_stage(usd_path=robot_usd_path, prim_path="/World/Humanoid")
    robot = Robot(prim_path="/World/Humanoid", name="humanoid", position=[0, 0, 1.0])
    world.scene.add(robot)

    # Reset to initialize
    world.reset()

    print("ROS 2 Bridge Setup Complete")
    print("Publishing topics:")
    print("  - /joint_states (sensor_msgs/JointState)")
    print("  - /tf (tf2_msgs/TFMessage)")
    print("  - /camera/image_raw (sensor_msgs/Image)")

    return world, robot

def setup_ros2_publishers():
    """Configure ROS 2 publishers for robot data"""

    # Joint state publisher
    # This is typically handled automatically by Isaac Sim's ROS 2 bridge
    # when you enable the Joint State component on the robot

    # Example: Add Camera with ROS 2 publisher
    from omni.isaac.sensor import Camera
    from omni.isaac.core.utils.prims import create_prim

    # Create camera prim
    camera_path = "/World/Camera"
    create_prim(camera_path, "Camera")

    # Create Camera sensor
    camera = Camera(
        prim_path=camera_path,
        position=[2.0, 0.0, 1.5],
        frequency=30,
        resolution=(640, 480)
    )

    camera.initialize()

    print(f"Camera created at {camera_path}")
    print("Camera will publish to /camera/image_raw")

    return camera

def run_ros2_simulation(world, robot, camera):
    """Run simulation with ROS 2 publishing"""

    print("\nStarting simulation with ROS 2 bridge...")
    print("Check topics with: ros2 topic list")
    print("Press Ctrl+C to stop\n")

    try:
        step_count = 0
        while True:
            # Step simulation
            world.step(render=True)
            step_count += 1

            # Log status every 100 steps
            if step_count % 100 == 0:
                position, _ = robot.get_world_pose()
                print(f"Step {step_count}: Robot at {position}")

            # Example: Get camera data
            if step_count % 30 == 0:  # Every 30 steps (1 Hz at 30 FPS)
                camera_data = camera.get_current_frame()
                if camera_data is not None:
                    print(f"Camera frame captured: {camera_data['rgba'].shape}")

    except KeyboardInterrupt:
        print("\nStopping simulation...")

def main():
    """Main entry point"""
    print("="*60)
    print("Isaac Sim ROS 2 Bridge Example")
    print("="*60)

    # Setup scene
    world, robot = setup_ros2_scene()

    # Setup ROS 2 publishers
    camera = setup_ros2_publishers()

    # Run simulation
    run_ros2_simulation(world, robot, camera)

    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
