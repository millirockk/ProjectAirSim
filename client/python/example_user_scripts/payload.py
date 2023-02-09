"""
Copyright (C) Microsoft Corporation. All rights reserved.

Demonstrates a payload actor with fast-physics.
"""

import asyncio

from projectairsim import ProjectAirSimClient, World, PayloadActor, Drone
from projectairsim.utils import projectairsim_log
from projectairsim.image_utils import ImageDisplay


# Async main function to wrap async drone commands
async def main():
    # Create a Project AirSim client
    client = ProjectAirSimClient()

    # Initialize an ImageDisplay object to display camera sub-windows
    image_display = ImageDisplay()

    try:
        # Connect to simulation environment
        client.connect()

        # Create a World object to interact with the sim world and load a scene
        world = World(client, "scene_payload_actor.jsonc", delay_after_load_sec=2)

        # Create a Drone object to interact with a drone in the loaded sim world
        drone = Drone(client, world, "Drone1")

        # Create PayloadActor object
        payload = PayloadActor(
            client, world, "Payload"
        )  # spawn like drone in config and watch it fall

        # ------------------------------------------------------------------------------

        # Subscribe to chase camera sensor as a client-side pop-up window
        chase_cam_window = "ChaseCam"
        image_display.add_chase_cam(chase_cam_window)
        client.subscribe(
            drone.sensors["Chase"]["scene_camera"],
            lambda _, chase: image_display.receive(chase, chase_cam_window),
        )

        # Subscribe to the downward-facing camera sensor's RGB and Depth images
        rgb_name = "RGB-Image"
        image_display.add_image(rgb_name, subwin_idx=0)
        client.subscribe(
            drone.sensors["DownCamera"]["scene_camera"],
            lambda _, rgb: image_display.receive(rgb, rgb_name),
        )

        image_display.start()

        # ------------------------------------------------------------------------------

        # Set the drone to be ready to fly
        drone.enable_api_control()
        drone.arm()

        # ------------------------------------------------------------------------------

        projectairsim_log().info("takeoff_async: starting")
        takeoff_task = await drone.takeoff_async()

        await takeoff_task
        projectairsim_log().info("takeoff_async: completed")

        # Command the drone to move up in NED coordinate system at 1 m/s for 3 seconds
        move_up_task = await drone.move_by_velocity_async(
            v_north=0.0, v_east=0.0, v_down=-1.0, duration=2.0
        )
        projectairsim_log().info("Move-Up invoked")

        await move_up_task
        projectairsim_log().info("Move-Up completed")

        # Command the drone to move to the payload position
        payload_pos = payload.get_kinematics()["pose"]["position"]

        projectairsim_log().info("Moving to payload position")
        move_to_payload_task = await drone.move_to_position_async(
            north=payload_pos["x"], east=payload_pos["y"], down=-3, velocity=1.0
        )
        await move_to_payload_task
        projectairsim_log().info("Moved to payload")

        # Command the drone to attach payload
        projectairsim_log().info("Attaching payload")
        success = drone.attach_payload(payload.name)
        projectairsim_log().info(f"Payload was successfully attached: {success}")

        # Command the drone to move up in NED coordinate system at 1 m/s for 2 seconds
        move_up_task = await drone.move_by_velocity_async(
            v_north=0.0, v_east=0.0, v_down=-1.0, duration=2.0
        )
        projectairsim_log().info("Move-Up invoked")
        await move_up_task
        projectairsim_log().info("Move-Up completed")

        # Command the drone to move north in NED coordinate system at 1 m/s for 2 seconds
        move_north_task = await drone.move_by_velocity_async(
            v_north=1.0, v_east=1.0, v_down=0.0, duration=4.0
        )
        projectairsim_log().info("Move-North invoked")
        await move_north_task
        projectairsim_log().info("Move-North completed")

        # Command the drone to move down in NED coordinate system at 1 m/s for 3.0 seconds
        move_down_task = await drone.move_by_velocity_async(
            v_north=0.0, v_east=0.0, v_down=1.0, duration=3.0
        )
        projectairsim_log().info("Move-Down invoked")
        await move_down_task
        projectairsim_log().info("Move-Down completed")

        await asyncio.sleep(1)

        # Command the drone to lower payload
        projectairsim_log().info("Lowering payload")
        success = drone.lower_payload(15)
        projectairsim_log().info(f"Payload was successfully lowered: {success}")

        # Command the drone to detach payload
        projectairsim_log().info("Detaching payload")
        success = drone.detach_payload()
        projectairsim_log().info(f"Payload was successfully detached: {success}")

        # Command the drone to move north-east in NED coordinate system at 1 m/s for 1 second
        move_north_task = await drone.move_by_velocity_async(
            v_north=1.0, v_east=1.0, v_down=0.0, duration=2.0
        )
        projectairsim_log().info("Move-North invoked")
        await move_north_task
        projectairsim_log().info("Move-North completed")

        # Command the drone to move down in NED coordinate system at 2 m/s for 3 seconds
        move_down_task = await drone.move_by_velocity_async(
            v_north=0.0, v_east=0.0, v_down=2.0, duration=2.0
        )
        projectairsim_log().info("Move-Down invoked")
        await move_down_task
        projectairsim_log().info("Move-Down completed")

        projectairsim_log().info("land_async: starting")
        land_task = await drone.land_async()
        await land_task
        projectairsim_log().info("land_async: completed")

        # ------------------------------------------------------------------------------

        # Shut down the drone
        drone.disarm()
        drone.disable_api_control()

        # ------------------------------------------------------------------------------

    # logs exception on the console
    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)

    finally:
        # Always disconnect from the simulation environment to allow next connection
        client.disconnect()
        image_display.stop()


if __name__ == "__main__":
    asyncio.run(main())  # Runner for async main function
