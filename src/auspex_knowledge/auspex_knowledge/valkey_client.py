#!/usr/bin/env python3
import json
import valkey as redis


class ValkeyClient:
    def __init__(self):
        self._redis_client = redis.Redis(
            host='localhost',
            port=6379,
            decode_responses=True,
            password='auspex_db',
            socket_connect_timeout=10
        )
        self._is_db_connected = False
        try:
            self._redis_client.ping()
            self._is_db_connected = True
        except redis.ConnectionError:
            print('error: failed to connect to valkey')
            return

    def _check_db_preconditions(self):
        if not self._is_db_connected:
            print('error: not connected to valkey')
            return False
        return True

    def create_collection(self, name):
        if not self._redis_client.exists(name):
            self._redis_client.json().set(name, '$', [])

    def append(self, collection, path, entity):
        if not self._check_db_preconditions():
            return False
        try:
            self._redis_client.json().arrappend(collection, path, entity)
            return True
        except redis.ResponseError as response_error:
            print('error: ', response_error, collection, path, entity)
            return False

    def query(self, collection, path):
        if not self._check_db_preconditions():
            return None
        try:
            result = self._redis_client.json().get(collection, path)
            return result
        except redis.ResponseError as response_error:
            print('error: ', response_error, collection, path)
            return None

    def set(self, collection, path, value):
        if not self._check_db_preconditions():
            return False
        try:
            self._redis_client.json().set(collection, path, value)
            return True
        except redis.ResponseError as response_error:
            print('error: ', response_error, collection, path, value)
            return False

    def delete(self, collection, path):
        if not self._check_db_preconditions():
            return False
        try:
            self._redis_client.json().delete(collection, path)
            return True
        except redis.ResponseError as response_error:
            print('error: ', response_error, collection, path)
            return False

    def save(self, filename):
        cursor = 0
        collections = []
        while True:
            cursor, keys = self._redis_client.scan(cursor=cursor, match="*")
            collections.extend(keys)
            if cursor == 0:
                break

        db_dump = {}

        for collection in collections:
            db_dump[collection] = self.query(collection, '$')

        with open(filename, 'w') as file:
            json.dump(db_dump, file, indent=2)

    def drop(self):
        try:
            self._redis_client.flushall()
        except Exception as e:
            print('error: ', e)

    def disconnect(self):
        try:
            self._redis_client.close()
        except Exception as e:
            print('error: ', e)
