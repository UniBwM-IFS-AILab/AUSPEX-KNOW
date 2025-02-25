#!/usr/bin/env python3
import json
from rclpy.node import Node
from auspex_msgs.srv import  ExistsKnowledge, InsertKnowledge, QueryKnowledge, UpdateKnowledge, DeleteKnowledge, WriteKnowledge


class WorldKnowledgeServer(Node):
    def __init__(self, wkb):
        super().__init__('world_knowledge_server')
        self.exists_srv = self.create_service(ExistsKnowledge, 'exists_knowledge', self.exists_knowledge)
        self.insert_srv = self.create_service(InsertKnowledge, 'insert_knowledge', self.insert_knowledge)
        self.query_srv = self.create_service(QueryKnowledge, 'query_knowledge', self.query_knowledge)
        self.update_srv = self.create_service(UpdateKnowledge, 'update_knowledge', self.update_knowledge)
        self.delete_srv = self.create_service(DeleteKnowledge, 'delete_knowledge', self.delete_knowledge)
        self.write_srv = self.create_service(WriteKnowledge, 'write_knowledge', self.write_knowledge)
        self._wkb = wkb

    def exists_knowledge(self, request, response):
        response.exists = self._wkb.exists(request.collection, request.path)
        return response

    def insert_knowledge(self, request, response):
        entity = {}
        try:
            entity = json.loads(request.entity)
        except json.JSONDecodeError:
            print('error: not a valid json')
            response.success = False
            return response
        response.success = self._wkb.insert(request.collection, request.path, entity)
        return response

    def query_knowledge(self, request, response):
        response.answer = self._wkb.query(request.collection, request.path)
        return response

    def update_knowledge(self, request, response):
        response.success = self._wkb.update(request.collection, request.path, request.value)
        return response

    def write_knowledge(self, request, response):
        entity = {}
        try:
            entity = json.loads(request.entity)
        except json.JSONDecodeError:
            print('error: not a valid json')
            response.success = False
            return response

        exists = self._wkb.exists(request.collection, request.path)
        if not exists:
            response.success = self._wkb.insert(request.collection, '$', entity)
        else:
            response.success = self._wkb.update(request.collection, request.path, entity)
        return response

    def delete_knowledge(self, request, response):
        response.success = self._wkb.delete(request.collection, request.path)
        return response
