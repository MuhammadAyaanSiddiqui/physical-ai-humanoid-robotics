#!/usr/bin/env python3
"""
Basic Isaac Sim scene launcher
Loads a warehouse environment with Carter robot
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim headless or with GUI
simulation_app = SimulationApp({"headless": False})

import omni
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
import numpy as np

def main():
    # Create world
    world = World(stage_units_in_meters=1.0)

    # Load warehouse environment
    omni.usd.get_context().open_stage(
        "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    )

    # Add Carter robot
    carter = world.scene.add(
        Robot(
            prim_path="/World/Carter",
            name="carter",
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/Carter/carter_v1.usd",
            position=np.array([0, 0, 0])
        )
    )

    # Reset world
    world.reset()

    print("Isaac Sim scene ready!")
    print(f"Carter robot loaded at {carter.get_world_pose()}")

    # Simulation loop
    for i in range(1000):
        world.step(render=True)

        if i % 100 == 0:
            print(f"Simulation step: {i}")

    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
