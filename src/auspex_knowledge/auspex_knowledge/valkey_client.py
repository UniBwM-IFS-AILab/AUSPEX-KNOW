#!/usr/bin/env python3
import threading
import valkey as redis


class ValkeyClient:
    def __init__(self, db_ip ="127.0.0.1"):
        self._redis_client = redis.Redis(
            host=db_ip,
            port=6379,
            decode_responses=True,
            password="auspex_db",
            socket_connect_timeout=10
        )
        self._is_db_connected = False
        self._observation_ready = False
        try:
            self._redis_client.ping()
            self._is_db_connected = True
        except redis.ConnectionError:
            print("error: failed to connect to valkey")
            return

    def exists(self, namespace, key):
        try:
            ret = self._redis_client.exists(f"{namespace}:{key}")
            return ret == 1
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}")
            return False

    def expire(self, namespace, key, ttl):
        try:
            ret = self._redis_client.expire(f"{namespace}:{key}", ttl)
            return ret == 1
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}")
            return False

    def set(self, namespace, key, value):
        try:
            self._redis_client.set(f"{namespace}:{key}", value)
            return True
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}", value)
            return False

    def get(self, namespace, key):
        try:
            return self._redis_client.get(f"{namespace}:{key}")
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}")
            return None

    def delete(self, namespace, key):
        try:
            self._redis_client.delete(f"{namespace}:{key}")
            return True
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}")
            return False

    def arrappend(self, namespace, key, path, entity):
        try:
            self._redis_client.json().arrappend(f"{namespace}:{key}", path, entity)
            return True
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}", path, entity)
            return False

    def jsonget(self, namespace, key, path):
        try:
            result = self._redis_client.json().get(f"{namespace}:{key}", path)
            if isinstance(result, list) and path == "$":
                return result[0] if result else None
            return result
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}", path)
            return None

    def jsonset(self, namespace, key, path, value):
        try:
            self._redis_client.json().set(f"{namespace}:{key}", path, value)
            return True
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}", path, value)
            return False

    def jsondelete(self, namespace, key, path):
        try:
            self._redis_client.json().delete(f"{namespace}:{key}", path)
            return True
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}", path)
            return False

    def sadd(self, namespace, key, value):
        try:
            self._redis_client.sadd(f"{namespace}:{key}", value)
            return True
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}", value)
            return False

    def smembers(self, namespace, key):
        try:
            return self._redis_client.smembers(f"{namespace}:{key}")
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}")
            return set()

    def srem(self, namespace, key, value):
        try:
            self._redis_client.srem(f"{namespace}:{key}", value)
            return True
        except redis.ResponseError as response_error:
            print("error: ", response_error, f"{namespace}:{key}", value)
            return False

    def hset(self, namespace, key, value):
        try:
            self._redis_client.hset(namespace, key, value)
            return True
        except redis.ResponseError as response_error:
            print("error: ", response_error, namespace, key, value)
            return False

    def hget(self, namespace, key):
        try:
            val = self._redis_client.hget(namespace, key)
            return val
        except redis.ResponseError as response_error:
            print("error: ", response_error, namespace, key)
            return None

    def hgetall(self, namespace):
        try:
            vals = self._redis_client.hgetall(namespace)
            return vals
        except redis.ResponseError as response_error:
            print("error: ", response_error, namespace)
            return {}

    def hdel(self, namespace, key):
        try:
            self._redis_client.hdel(namespace, key)
            return True
        except redis.ResponseError as response_error:
            print("error: ", response_error, namespace, key)
            return False

    def scan_keys(self, pattern):
        try:
            cursor = 0
            keys = []
            while True:
                cursor, batch = self._redis_client.scan(cursor=cursor, match=pattern)
                keys.extend(batch)
                if cursor == 0:
                    break
            return keys
        except redis.ResponseError as response_error:
            print("error: ", response_error, pattern)
            return []

    def init_observations(self, observe_callback, expire_callback):
        if self._observation_ready:
            print("observations already initialized.")
            return
        self._pubsub = self._redis_client.pubsub(ignore_subscribe_messages=True)
        self._observe_callback = observe_callback
        self._expire_callback = expire_callback

        self._pubsub.psubscribe("__keyevent@0__:expired")

        self._observation_ready = True

        self._listen_thread = threading.Thread(target=self._observing_loop, daemon=True)
        self._listen_thread.start()

    def _observing_loop(self):
        while self._observation_ready:
            msg = self._pubsub.get_message(ignore_subscribe_messages=True, timeout=1)
            if msg and msg.get("type") == "pmessage":
                channel = msg["channel"]

                if channel.startswith("__keyevent@"):
                    key = msg["data"]
                    self._expire_callback(key)

                else:
                    key = channel.split(":", 1)[-1]
                    self._observe_callback(key)

    def stop_observations(self):
        self._observation_ready = False
        if hasattr(self, "_pubsub"):
            self._pubsub.close()

    def observe(self, key):
        try:
            channel = f"__keyspace@0__:{key}"
            self._pubsub.psubscribe(channel)
        except Exception as e:
            print("failed to subscribe: ", e)

    def unobserve(self, key):
        try:
            channel = f"__keyspace@0__:{key}"
            self._pubsub.punsubscribe(channel)
        except Exception as e:
            print("failed to unsubscribe: ", e)

    def get_memory_usage(self):
        info = self._redis_client.info("memory")
        #used_memory = info["used_memory"]
        used_memory_human = info["used_memory_human"]

        return used_memory_human

    def drop(self):
        try:
            self._redis_client.flushall()
        except Exception as e:
            print("error: ", e)

    def disconnect(self):
        try:
            self._redis_client.close()
        except Exception as e:
            print("error: ", e)
