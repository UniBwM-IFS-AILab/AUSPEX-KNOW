#!/usr/bin/env python3
import json
import os
from rclpy.node import Node
import rosidl_runtime_py
from rclpy.qos import qos_profile_sensor_data

from auspex_msgs.msg import PlatformState, ObjectKnowledge, PlatformCapabilities, SearchMission

class KnowledgeCollector(Node):
    def __init__(self, wkb):
        super().__init__('knowledge_collector')

        self._params_dir = os.getenv('AUSPEX_PARAMS_PATH')

        self.sub_mission = self.create_subscription(
            SearchMission,
            'search_mission',
            self.search_mission_callback,
            qos_profile_sensor_data
        )
        self.sub_capabilities = self.create_subscription(
            PlatformCapabilities,
            'platform_capabilities',
            self.platform_capabilities_callback,
            qos_profile_sensor_data
        )
        self.sub_drone_state = self.create_subscription(
            PlatformState,
            'platform_state',
            self.drone_state_callback,
            qos_profile_sensor_data
        )
        self.sub_object_know = self.create_subscription(
            ObjectKnowledge,
            'detections',
            self.object_knowledge_callback,
            qos_profile_sensor_data
        )

        self._wkb = wkb

        self._wkb.drop()

        self.load_collections()

        self.load_configs()

        self.load_areas()

        self.configure_save()

        self.publish_timer = self.create_timer(1.0, self.write_history)

        # QUICK FIX for Bjoern
        self.expire_timer = self.create_timer(10.0, self.platform_expire)

    def load_collections(self):
        config_dir = os.path.join(self._params_dir, "config")
        collections_file = os.path.join(config_dir, "collections.json")

        with open(collections_file, "r") as file:
            collections = json.load(file)

        for name in collections:
            self._wkb.create_collection(name)

    def load_configs(self):
        config_dir = os.path.join(self._params_dir, "config")

        for config_file in os.listdir(config_dir):
            if config_file.endswith("_config.json"):
                collection_name = config_file.replace(".json", "")
                file_path = os.path.join(config_dir, config_file)

                with open(file_path, 'r') as file:
                    configs = json.load(file)

                for config in configs:
                    self._wkb.insert(collection_name, "$", entity=config)

    def configure_save(self):
        bsave = self._wkb.query("know_config","$.[0].save")
        if bsave:
            self.bsave = bsave[0]
        interval = self._wkb.query("know_config","$.[0].interval")
        if interval:
            self.interval = int(interval[0])
        filename = self._wkb.query("know_config","$.[0].file")
        if filename:
            self.filename = filename[0]

        if self.bsave == "true":
            print(f"saving knowledge base every {str(self.interval)}s to {self.filename}")
            self.publish_timer = self.create_timer(self.interval, self.save)

    def save(self):
        self._wkb.save(self.filename)

    def search_mission_callback(self, msg):
        msg_dict = rosidl_runtime_py.convert.message_to_ordereddict(msg)
        unique_val = str(msg_dict['team_id'])
        val_exists = self._wkb.exists('mission', '$[?(@.team_id=="' + unique_val + '")]')
        if not val_exists:
            self._wkb.insert('mission', '$', msg_dict)
        else:
            self._wkb.update('mission','$[?(@.team_id=="' + unique_val + '")]', msg_dict)
        return

    def platform_capabilities_callback(self, msg):
        msg_dict = rosidl_runtime_py.convert.message_to_ordereddict(msg)
        unique_val = str(msg_dict['platform_id'])
        val_exists = self._wkb.exists('capabilities', '$[?(@.platform_id=="' + unique_val + '")]')
        if not val_exists:
            self._wkb.insert('capabilities', '$', msg_dict)
        else:
            self._wkb.update('capabilities','$[?(@.platform_id=="' + unique_val + '")]', msg_dict)
        return

    def drone_state_callback(self, msg):
        msg_dict = rosidl_runtime_py.convert.message_to_ordereddict(msg)
        unique_val = 'platform_id'
        path = '$[?(@.' + unique_val + '=="' + str(msg_dict[unique_val]) + '")]'
        self._wkb.upsert('platform', path, msg_dict)

    def object_knowledge_callback(self, msg):
        msg_dict = rosidl_runtime_py.convert.message_to_ordereddict(msg)
        unique_val = str(msg_dict['id'])
        val_exists = self._wkb.exists('object', '$[?(@.id=="' + unique_val + '")]')
        if not val_exists:
            self._wkb.insert('object', '$', msg_dict)
        else:
            self._wkb.update('object','$[?(@.id=="' + unique_val + '")]', msg_dict)
        return

    def write_history(self):
        answer = self._wkb.query('platform', '$')
        if not answer:
            return
        try:
            json_str = answer[0].replace("'", '"')
            platforms = json.loads(json_str)
        except (json.JSONDecodeError, TypeError):
            print('error: malformed platform entity')
            return
        for platform in platforms:
            if 'platform_id' in platform:
                platform_id = platform['platform_id']
                if not self._wkb.exists('platform_history', '$[*].' + platform_id):
                    self._wkb.insert('platform_history', '$', {platform_id:[]})
                self._wkb.insert('platform_history', '$[*].' + platform_id, platform)

    def load_areas(self):
        file_path = os.path.join(self._params_dir, 'geographic', 'areas', 'areas.json')
        with open(file_path, 'r') as file:
            areas_json = json.load(file)

        self._wkb.update('area', '$', [])

        for area in areas_json:
            self._wkb.insert('area', '$', area)

    # QUICK FIX for Bjoern
    def platform_expire(self):
        answer = self._wkb.query('platform', '$')
        if not answer:
            return
        try:
            json_str = answer[0].replace("'", '"')
            platforms = json.loads(json_str)
        except (json.JSONDecodeError, TypeError):
            print('error: malformed platform entity')
            return
        for platform in platforms:
            if 'platform_id' in platform:
                platform_id = platform['platform_id']
                last_update_secs = int(platform['header']['stamp']['sec'])
                current_secs = self.get_clock().now().seconds_nanoseconds()[0]
                if current_secs - last_update_secs > 50:
                    print("platform " + platform_id + " expired.")
                    self._wkb.delete('platform','$[?(@.platform_id=="' + platform_id + '")]')
