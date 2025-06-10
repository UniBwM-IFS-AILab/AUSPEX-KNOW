#!/usr/bin/env python3
from rclpy.node import Node
from .valkey_client import ValkeyClient

from msg_context.loader import KnowledgeChange


class WorldKnowledgeBase(Node):
    def __init__(self):
        super().__init__('world_knowledge_base')
        self._valkey = ValkeyClient()

        self._publisher = self.create_publisher(
            KnowledgeChange,
            'knowledge_change',
            10
        )

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
        if success:
            self._publish_change(collection, path)
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
        success = self._valkey.set(collection, path, value)
        if success:
            self._publish_change(collection, path)
        return success

    def delete(self, collection, path):
        success = self._valkey.delete(collection, path)
        return success

    def _publish_change(self, collection, path):
        msg = KnowledgeChange()
        msg.collection = collection
        msg.path = path
        self._publisher.publish(msg)

    def _stringify(self, entity):
        if isinstance(entity, dict):
            return {key: self._stringify(value) for key, value in entity.items()}
        elif isinstance(entity, list):
            return [self._stringify(item) for item in entity]
        else:
            return str(entity)
