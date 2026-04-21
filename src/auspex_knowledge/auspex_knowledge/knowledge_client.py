#!/usr/bin/env python3
import os

from auspex_knowledge.knowledge_base import KnowledgeBase

def get_db_ip():
    return os.getenv("AUSPEX_DB_IP", "127.0.0.1")

class KnowledgeClient:
    def __init__(self, db_ip="127.0.0.1"):
        self._know_base = KnowledgeBase(db_ip)

    def read_slot(self, frame, instance_id, subframe, slot):
        value = self._know_base.read_slot(frame, instance_id, subframe, slot)
        return value

    def set_slot(self, frame, instance_id, subframe, slot, value, path="$", variant="main"):
        return self._know_base.set_slot(frame, instance_id, subframe, slot, value, path, variant)

    def delete_slot(self, frame, instance_id, subframe, slot, variant="main"):
        return self._know_base.delete_slot(frame, instance_id, subframe, slot, variant)

    def upsert_subframe(self, frame, instance_id, subframe, item):
        return self._know_base.upsert_subframe(frame, instance_id, subframe, item)

    def delete_subframe(self, frame, instance_id, subframe, variant="main"):
        return self._know_base.delete_subframe(frame, instance_id, subframe, variant)

    def get_instance_ids(self, frame):
         return self._know_base.get_instances(frame)

    def get_instance(self, frame, instance_id):
        return self._know_base.get_instance_json(frame, instance_id)

    def get_all_instances(self, frame):
        instance_ids = self._know_base.get_instances(frame)
        instance_jsons = []
        for instance_id in instance_ids:
            instance_jsons.append(self._know_base.get_instance_json(frame, instance_id))
        return instance_jsons

    def find_instance_ids(self, frame, subframe, slot, value):
        instance_ids = self._know_base.get_instances(frame)
        selected_ids = []
        for instance_id in instance_ids:
            slot_value = self._know_base.read_slot(frame, instance_id, subframe, slot)
            if slot_value == value:
                selected_ids.append(instance_id)
        return selected_ids

    def find_instances(self, frame, subframe, slot, value):
        instance_ids = self._know_base.get_instances(frame)
        instance_jsons = []
        for instance_id in instance_ids:
            slot_value = self._know_base.read_slot(frame, instance_id, subframe, slot)
            if slot_value == value:
                instance_jsons.append(self._know_base.get_instance_json(frame, instance_id))
        return instance_jsons

    def delete_instance(self, frame, instance_id):
        self._know_base.delete_instance(frame, instance_id)

    def delete_all_instances(self, frame):
        instance_ids = self._know_base.get_instances(frame)
        for instance_id in instance_ids:
            self._know_base.delete_instance(frame, instance_id)

    def delete_instances(self, frame, subframe, slot, value):
        instance_ids = self._know_base.get_instances(frame)
        for instance_id in instance_ids:
            slot_value = self._know_base.read_slot(frame, instance_id, subframe, slot)
            if slot_value == value:
                self._know_base.delete_instance(frame, instance_id)
