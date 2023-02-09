"""
Copyright (C) Microsoft Corporation. All rights reserved.
Python client for ProjectAirSim payload actors.
"""

from projectairsim import ProjectAirSimClient, World
from projectairsim.utils import projectairsim_log
from typing import Dict


class PayloadActor(object):
    def __init__(self, client: ProjectAirSimClient, world: World, name: str):
        """ProjectAirSim Payload Actor Interface

        Arguments:
            client {ProjectAirSimClient} -- ProjectAirSim client object
            world {World} -- ProjectAirSim world object
            name {str} -- Name of the payload actor in the scene
        """
        projectairsim_log().info(f"Initalizing PayloadActor '{name}'...")
        self.client = client
        self.name = name
        self.world_parent_topic = world.parent_topic
        self.set_topics(world)
        self.log_topics()
        projectairsim_log().info(
            f"Payload actor '{self.name}' initialized for "
            f"World scene '{self.world_parent_topic}'"
        )

    def set_topics(self, world: World):
        self.parent_topic = f"{self.world_parent_topic}/payload_actors/{self.name}"
        self.set_actor_info_topics()

    def set_actor_info_topics(self):
        self.actor_info = {}
        self.actor_info["actual_pose"] = f"{self.parent_topic}/actual_pose"

    def log_topics(self):
        projectairsim_log().info("-------------------------------------------------")
        projectairsim_log().info(
            f"The following topics can be subscribed to for actor '{self.name}':",
        )
        for name in self.actor_info.keys():
            projectairsim_log().info(f'    actor_info["{name}"]')
        projectairsim_log().info("-------------------------------------------------")

    def get_kinematics(self) -> Dict:
        """Get kinematics for payload object

        Returns:
            Dict: the kinematics
        """
        get_kinematics_req = {
            "method": f"{self.world_parent_topic}/GetPayloadKinematics",
            "params": {"payload_actor_name": self.name},
            "version": 1.0,
        }
        kinematics = self.client.request(get_kinematics_req)
        return kinematics
