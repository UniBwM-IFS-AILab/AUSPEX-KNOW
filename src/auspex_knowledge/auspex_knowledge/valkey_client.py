#!/usr/bin/env python3
import json
import valkey as redis


class ValkeyClient:
    def __init__(self):
        self._redis_client = redis.Redis(
            host='localhost',
            port=6379,
            decode_responses=True
        )
        self._is_db_connected = False
        try:
            self._redis_client.ping()
            self._is_db_connected = True
        except redis.ConnectionError:
            print('error: failed to connect to valkey')
            return

        self.drop()
        self._collections = ['platform','capabilities','plan','mission','object','geographic','config','history']
        for collection in self._collections:
            self._redis_client.json().set(collection, '$', [])

    def _check_db_preconditions(self, collection):
        if not self._is_db_connected:
            print('error: not connected to valkey')
            return False
        if collection not in self._collections:
            print('error: unknown collection key')
            return False
        return True

    def append(self, collection, path, entity):
        if not self._check_db_preconditions(collection):
            return False
        try:
            self._redis_client.json().arrappend(collection, path, entity)
            return True
        except redis.ResponseError as response_error:
            print('error: ', response_error, collection, path, entity)
            return False

    def query(self, collection, path):
        if not self._check_db_preconditions(collection):
            return None
        try:
            result = self._redis_client.json().get(collection, path)
            return result
        except redis.ResponseError as response_error:
            print('error: ', response_error, collection, path)
            return None

    def set(self, collection, path, value):
        if not self._check_db_preconditions(collection):
            return False
        try:
            self._redis_client.json().set(collection, path, value)
            return True
        except redis.ResponseError as response_error:
            print('error: ', response_error, collection, path, value)
            return False

    def delete(self, collection, path):
        if not self._check_db_preconditions(collection):
            return False
        try:
            self._redis_client.json().delete(collection, path)
            return True
        except redis.ResponseError as response_error:
            print('error: ', response_error, collection, path)
            return False

    def save_to_file(self, filename):
        db_dump = {}

        for collection in self._collections:
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

    def __del__(self):
        self.save_to_file('wk.json')
        self.drop()
        self.disconnect()
