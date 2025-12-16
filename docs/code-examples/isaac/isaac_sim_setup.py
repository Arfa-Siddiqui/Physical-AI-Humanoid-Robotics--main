#!/usr/bin/env python3
"""
Isaac Sim Basic Setup Script
Creates a simple scene with a humanoid robot in NVIDIA Isaac Sim

Prerequisites:
- Isaac Sim 2023.1.0 or later installed
- Run from Isaac Sim Python: ./python.sh isaac_sim_setup.py

Documentation: https://docs.omniverse.nvidia.com/isaacsim/
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

import omni
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
import carb

def setup_scene():
    """Create Isaac Sim scene with humanoid robot"""

    # Create world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Get Nucleus assets path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return

    print(f"Assets root path: {assets_root_path}")

    # Add humanoid robot (using Isaac Sim's built-in humanoid)
    # Alternative: Use custom URDF via omni.isaac.core.utils.stage.add_reference_to_stage
    robot_usd_path = f"{assets_root_path}/Isaac/Robots/Humanoid/humanoid.usd"

    add_reference_to_stage(
        usd_path=robot_usd_path,
        prim_path="/World/Humanoid"
    )

    # Create Robot instance for control
    robot = Robot(
        prim_path="/World/Humanoid",
        name="humanoid_robot",
        position=[0.0, 0.0, 1.0]  # 1m above ground
    )

    world.scene.add(robot)

    # Add lighting
    from omni.isaac.core.utils.prims import create_prim
    create_prim(
        "/World/Light",
        "DistantLight",
        attributes={"intensity": 500.0}
    )

    print("Scene setup complete")
    print("Robot:", robot.name)
    print("Position:", robot.get_world_pose()[0])

    # Reset world to initialize physics
    world.reset()

    return world, robot

def run_simulation(world, robot):
    """Run simulation loop"""
    print("Starting simulation...")
    print("Press Ctrl+C to stop")

    try:
        # Simulation loop
        for i in range(1000):
            # Step physics
            world.step(render=True)

            if i % 100 == 0:
                # Get robot state every 100 steps
                position, orientation = robot.get_world_pose()
                print(f"Step {i}: Robot position = {position}")

            # Example: Apply joint torques (uncomment to test)
            # joint_positions = robot.get_joint_positions()
            # # Set desired joint velocities or positions here

    except KeyboardInterrupt:
        print("\nSimulation stopped by user")

def main():
    """Main entry point"""
    print("="*50)
    print("Isaac Sim Humanoid Setup")
    print("="*50)

    # Setup scene
    world, robot = setup_scene()

    # Run simulation
    run_simulation(world, robot)

    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
