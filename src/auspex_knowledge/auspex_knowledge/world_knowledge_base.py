#!/usr/bin/env python3
from rclpy.node import Node
from .valkey_client import ValkeyClient

class WorldKnowledgeBase(Node):
    def __init__(self):
        super().__init__('world_knowledge_base')
        self._valkey = ValkeyClient()

    def create_collection(self, name):
        self._valkey.create_collection(str(name))

    def exists(self, collection, path):
        answer = self._valkey.query(collection, path)
        if not answer:
            return False
        elif isinstance(answer, list) and not all(answer):
            return False
        else:
            return True

    def insert(self, collection, path, entity):
        str_entity = self._stringify(entity)
        success = self._valkey.append(collection, path, str_entity)
        return success

    def query(self, collection, path):
        answer = self._valkey.query(collection, path)
        if not isinstance(answer, list):
            answer = [str(answer)]
        else:
            answer_array = []
            for element in answer:
                answer_array.append(str(element))
            answer = answer_array
        return answer

    def update(self, collection, path, value):
        str_value = self._stringify(value)
        success = self._valkey.set(collection, path, str_value)
        return success

    def upsert(self, collection, path, entity):
        if not self.exists(collection, path):
            return self.insert(collection, '$', entity)
        else:
            return self.update(collection, path, entity)

    def delete(self, collection, path):
        success = self._valkey.delete(collection, path)
        return success

    def save(self, filename):
        self._valkey.save(filename)

    def drop(self):
        self._valkey.drop()

    def _stringify(self, entity):
        if isinstance(entity, dict):
            return {key: self._stringify(value) for key, value in entity.items()}
        elif isinstance(entity, list):
            return [self._stringify(item) for item in entity]
        else:
            return str(entity)
