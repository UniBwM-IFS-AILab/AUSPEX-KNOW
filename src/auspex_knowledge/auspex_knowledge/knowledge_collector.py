#!/usr/bin/env python3
import json
import os
from rclpy.node import Node
import rosidl_runtime_py
from rclpy.qos import qos_profile_sensor_data

from msg_context.loader import PlatformState, ObjectKnowledge, PlatformCapabilities, SearchMission

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
            'drone_state',
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

        self.load_configs()

        self.load_areas()

        self.publish_timer = self.create_timer(1.0, self.write_history)

    def load_configs(self):
        file_path = os.path.join(self._params_dir, 'config', 'planner_config.json')
        with open(file_path, 'r') as file:
            planner_configs = json.load(file)

        for planner_config in planner_configs:
            self._wkb.insert('config', '$', entity=planner_config)

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
        unique_val = str(msg_dict['platform_id'])
        self._wkb.update('platform','$[?(@.platform_id=="' + unique_val + '")]', msg_dict)
        return

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
                if 'team_id' in platform:
                    team_id = platform['team_id']
                    if not self._wkb.exists('history', '$[?(@.platform_id=="' + platform_id + '")]'):
                        history_entity = {'platform_id': str(platform_id), 'team_id': str(team_id), 'trajectory': []}
                        self._wkb.insert('history', '$', history_entity)
                    try:
                        stamp = platform['header']['stamp']['sec']
                        lat = platform['platform_gps_position']['latitude']
                        lon = platform['platform_gps_position']['longitude']
                        alt = platform['platform_gps_position']['altitude']
                    except KeyError:
                        print('error: unexpected json structure')
                    self._wkb.insert('history', '$[?(@.platform_id=="' + platform_id + '")].trajectory', {'stamp': str(stamp), 'lat': str(lat), 'lon': str(lon), 'alt': str(alt)})

    def load_areas(self):
        file_path = os.path.join(self._params_dir, 'geographic', 'areas', 'areas.json')
        with open(file_path, 'r') as file:
            areas_json = json.load(file)

        self._wkb.update('area', '$', [])

        for area in areas_json:
            self._wkb.insert('area', '$', area)
