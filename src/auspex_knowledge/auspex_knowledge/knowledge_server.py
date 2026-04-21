#!/usr/bin/env python3
import json

from rclpy.node import Node

from auspex_msgs.srv import ReadSlot, SetSlot, DeleteSlot
from auspex_msgs.srv import UpsertSubframe, DeleteSubframe
from auspex_msgs.srv import GetInstanceIds, GetInstance, GetAllInstances
from auspex_msgs.srv import FindInstanceIds, FindInstances
from auspex_msgs.srv import DeleteInstance, DeleteInstances, DeleteAllInstances

from auspex_knowledge.knowledge_client import KnowledgeClient

class KnowledgeServer(Node):
    def __init__(self):
        super().__init__("knowledge_server")
        self.read_slot_srv = self.create_service(ReadSlot, "read_slot", self.read_slot)
        self.set_slot_srv = self.create_service(SetSlot, "set_slot", self.set_slot)
        self.delete_slot_srv = self.create_service(DeleteSlot, "delete_slot", self.delete_slot)
        self.upsert_subframe_srv = self.create_service(UpsertSubframe, "upsert_subframe", self.upsert_subframe)
        self.delete_subframe_srv = self.create_service(DeleteSubframe, "delete_subframe", self.delete_subframe)
        self.get_instances_srv = self.create_service(GetInstanceIds, "get_instance_ids", self.get_instance_ids)
        self.get_instance_srv = self.create_service(GetInstance, "get_instance", self.get_instance)
        self.get_all_instances_srv = self.create_service(GetAllInstances, "get_all_instances", self.get_all_instances)
        self.find_instance_ids_srv = self.create_service(FindInstanceIds, "find_instance_ids", self.find_instance_ids)
        self.find_instances_srv = self.create_service(FindInstances, "find_instances", self.find_instances)
        self.delete_instance_srv = self.create_service(DeleteInstance, "delete_instance", self.delete_instance)
        self.delete_instances_srv = self.create_service(DeleteInstances, "delete_instances", self.delete_instances)
        self.delete_all_instances_srv = self.create_service(DeleteAllInstances, "delete_all_instances", self.delete_all_instances)

        self._know_client = KnowledgeClient()

    def read_slot(self, request, response):
        frame = request.frame
        instance_id = request.instance_id
        subframe = request.subframe
        slot = request.slot
        try:
            value = json.dumps(self._know_client.read_slot(frame, instance_id, subframe, slot))
        except TypeError:
            response.value = None
            print("read_slot: invalid JSON.")
            return response

        response.value = value
        return response

    def set_slot(self, request, response):
        frame = request.frame
        instance_id = request.instance_id
        subframe = request.subframe
        slot = request.slot
        value = request.value
        path = request.path
        variant = request.variant.strip()

        if not variant:
            variant="main"

        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            response.success = False
            print("set_slot: invalid JSON.")
            return response

        response.success = self._know_client.set_slot(frame, instance_id, subframe, slot, value, path, variant)
        return response

    def delete_slot(self, request, response):
        frame = request.frame
        instance_id = request.instance_id
        subframe = request.subframe
        slot = request.slot
        variant = request.variant.strip()

        if not variant:
            variant="main"

        response.success = self._know_client.delete_slot(frame, instance_id, subframe, slot, variant)
        return response

    def upsert_subframe(self, request, response):
        frame = request.frame
        instance_id = request.instance_id
        subframe = request.subframe
        item = request.item
        try:
            item_dict = json.loads(item)
        except json.JSONDecodeError:
            response.success = False
            print("upsert_subframe: invalid JSON.")
            return response

        response.success = self._know_client.upsert_subframe(frame, instance_id, subframe, item_dict)
        return response

    def delete_subframe(self, request, response):
        frame = request.frame
        instance_id = request.instance_id
        subframe = request.subframe
        variant = request.variant.strip()

        if not variant:
            variant="main"

        response.success = self._know_client.delete_subframe(frame, instance_id, subframe, variant)
        return response

    def get_instance_ids(self, request, response):
        frame = request.frame

        response.instance_ids= self._know_client.get_instance_ids(frame)
        return response

    def get_instance(self, request, response):
        frame = request.frame
        instance_id = request.instance_id
        instance =  self._know_client.get_instance(frame, instance_id)

        try:
            instance = json.dumps(instance)
        except TypeError:
            response.instance = None
            print("get_instance: invalid JSON.")
            return response

        response.instance = instance
        return response

    def get_all_instances(self, request, response):
        frame = request.frame
        instances = self._know_client.get_all_instances(frame)
        result_instances = []
        for instance in instances:
            try:
                instance_str = json.dumps(instance)
                result_instances.append(instance_str)
            except TypeError:
                print("get_all_instances: invalid JSON.")
                continue

        response.instances = result_instances
        return response

    def find_instance_ids(self, request, response):
        frame = request.frame
        subframe = request.subframe
        slot = request.slot
        value = request.value
        response.instance_ids =  self._know_client.find_instance_ids(frame, subframe, slot, value)
        return response

    def find_instances(self, request, response):
        frame = request.frame
        subframe = request.subframe
        slot = request.slot
        try:
            value = json.loads(request.value)
        except json.JSONDecodeError:
            response.instances = []
            print("find_instances: invalid JSON value.")
            return response

        instances = self._know_client.find_instances(frame, subframe, slot, value)
        result_instances = []
        for instance in instances:
            try:
                instance_str = json.dumps(instance)
                result_instances.append(instance_str)
            except TypeError:
                print("find_instances: invalid JSON.")
                continue

        response.instances = result_instances
        return response

    def delete_instance(self, request, response):
        frame = request.frame
        instance_id = request.instance_id
        self._know_client.delete_instance(frame, instance_id)
        response.success = True
        return response

    def delete_all_instances(self, request, response):
        frame = request.frame
        self._know_client.delete_all_instances(frame)
        response.success = True
        return response

    def delete_instances(self, request, response):
        frame = request.frame
        subframe = request.subframe
        slot = request.slot
        try:
            value = json.loads(request.value)
        except json.JSONDecodeError:
            response.success = False
            print("delete_instances: invalid JSON value.")
            return response
        self._know_client.delete_instances(frame, subframe, slot, value)
        response.success = True
        return response
