#!/usr/bin/env python3
import json
import yaml

from rclpy.node import Node

from auspex_knowledge.knowledge_base import KnowledgeBase

class KnowledgeLoader(Node):
    def __init__(self):
        super().__init__("knowledge_loader")

        self.config_file = "/root/AUSPEX/AUSPEX-KNOW/src/auspex_knowledge/auspex_knowledge/config/static_knowledge.yaml"

        self._know_base = KnowledgeBase()

        self._load_knowledge()

    def _load_knowledge(self):
        with open(self.config_file, "r") as file:
            config = yaml.safe_load(file)

        for frame, frame_config in config["frames"].items():
            id_field = frame_config["id_field"]
            file_name = frame_config["file"]

            with open(file_name, "r") as json_file:
                entities = json.load(json_file)

            for entity in entities:
                if id_field not in entity:
                    continue

                instance_id = entity[id_field]

                for slot, value in entity.items():
                    self._know_base.set_slot(frame, instance_id, "data", slot, value)
