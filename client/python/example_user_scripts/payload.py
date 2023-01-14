"""
Copyright (C) Microsoft Corporation. All rights reserved.

Demonstrates a payload actor with fast-physics.
"""

import asyncio

from projectairsim import ProjectAirSimClient, World, PayloadActor, Drone
from projectairsim.utils import projectairsim_log
from typing import List


# Async main function to wrap async drone commands
async def main():
    # Create a Project AirSim client
    client = ProjectAirSimClient()

    try:
        # Connect to simulation environment
        client.connect()

        # Create a World object to interact with the sim world and load a scene
        world = World(client, "scene_payload_actor.jsonc", delay_after_load_sec=2)

        # Create a Drone object to interact with a drone in the loaded sim world
        drone = Drone(client, world, "Drone1")

        # Create PayloadActor object
        payload = PayloadActor(client, world, "Payload") # spawn like drone in config and watch it fall


    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)

    finally:
        # Always disconnect from the simulation environment to allow next connection
        client.disconnect()


if __name__ == "__main__":
    asyncio.run(main())  # Runner for async main function
