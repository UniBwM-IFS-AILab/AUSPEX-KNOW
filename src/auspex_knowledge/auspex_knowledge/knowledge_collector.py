#!/usr/bin/env python3
import functools
import importlib
import yaml

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict

from auspex_knowledge.knowledge_base import KnowledgeBase


class KnowledgeCollector(Node):
    def __init__(self):
        super().__init__("knowledge_collector")

        self.config_file = "/root/AUSPEX/AUSPEX-KNOW/src/auspex_knowledge/auspex_knowledge/config/dynamic_knowledge.yaml"

        self._know_base = KnowledgeBase()

        self._qos_profile_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self._qos_profile_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self._init_collection()

    def _import_msg_type(self, type_str):
            pkg, _, name = type_str.partition("/msg/")
            module = importlib.import_module(f"{pkg}.msg")
            return getattr(module, name)

    def _init_collection(self):
        with open(self.config_file, "r") as file:
            config = yaml.safe_load(file)

        self._id_fields = {}
        self._subscribers = {}
        self._frame_map = {}

        for frame_name, frame_config in config["frames"].items():
            id_field = frame_config["id_field"]

            self._know_base.create_frame(frame_name, frame_config)

            for topic_name, topic_config in frame_config["topics"].items():
                topic = topic_config["topic"]
                msg_type_str = topic_config["msg_type"]

                self._frame_map[topic] = frame_name
                self._id_fields[topic] = id_field

                msg_type = self._import_msg_type(msg_type_str)
                callback = functools.partial(self._collect_callback, topic, topic_name)

                qos = topic_config.get("qos_profile", "reliable")

                if qos == "best_effort":
                    qos_profile = self._qos_profile_best_effort
                elif qos == "reliable":
                    qos_profile = self._qos_profile_reliable

                self._subscribers[topic] = self.create_subscription(
                    msg_type,
                    topic,
                    callback,
                    qos_profile
                )

    def _collect_callback(self, topic, topic_name, msg):
        msg_dict = message_to_ordereddict(msg)
        id_field = self._id_fields[topic]
        id = str(msg_dict[id_field])
        frame = self._frame_map[topic]
        self._know_base.upsert_subframe(frame, id, topic_name, msg_dict)
