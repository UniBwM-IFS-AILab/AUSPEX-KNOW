#!/usr/bin/env python3
import random

from auspex_knowledge.valkey_client import ValkeyClient


class KnowledgeBase():
    def __init__(self, db_ip="127.0.0.1"):
        self._valkey = ValkeyClient(db_ip)

    def create_frame(self, frame, frame_schema):
        for topic_name, topic_config in frame_schema["topics"].items():
            for property_name, value in topic_config.items():
                namespace = f"schema:{frame}:{topic_name}"
                self._valkey.set(namespace, property_name, str(value))

    def exists_instance(self, frame, instance_id):
        return instance_id in self.get_instances(frame)

    def get_instances(self, frame):
        return self._valkey.smembers(frame, "instances")

    def delete_instance(self, frame, instance_id):
        keys = self._valkey.scan_keys(f"{frame}:{instance_id}:*")

        for full_key in keys:
            namespace, subkey = full_key.split(":", 1)
            self._valkey.delete(namespace, subkey)

        self._valkey.srem(frame, "instances", instance_id)

        return True

    def get_variants(self, frame, instance_id, subframe):
        return self._valkey.smembers(f"{frame}:{instance_id}:{subframe}", "variants")

    def get_subframe(self, frame, instance_id, subframe, variant="main"):
        return self._valkey.jsonget(frame, f"{instance_id}:{subframe}:{variant}", "$")

    def get_subframe_schema(self, frame, subframe):
        mode = self._valkey.get(f"schema:{frame}:{subframe}", "mode") or "single"
        variant_key = self._valkey.get(f"schema:{frame}:{subframe}", "variant_key") or None
        ttl_raw = self._valkey.get(f"schema:{frame}:{subframe}", "ttl")
        ttl = int(ttl_raw) if ttl_raw not in (None, "") else 0
        return {"mode": mode, "variant_key": variant_key, "ttl": ttl}

    def get_variant(self, schema, item=None):
        if schema["mode"] == "multiple" and item is not None:
            return item.get(schema["variant_key"]) or "unknown_variant"
        return "main"

    def upsert_subframe(self, frame, instance_id, subframe, item):
        schema = self.get_subframe_schema(frame, subframe)
        variant = self.get_variant(schema, item)

        success = self._valkey.jsonset(frame, f"{instance_id}:{subframe}:{variant}", "$", item)

        ttl = schema["ttl"]
        if ttl:
            self._valkey.expire(frame, f"{instance_id}:{subframe}:{variant}", ttl)

        self._valkey.sadd(frame, "instances", instance_id)
        self._valkey.sadd(f"{frame}:{instance_id}:{subframe}", "variants", variant)

        slot_items = self._extract_slots(item)
        for slot, value in slot_items:
            self.set_slot(frame, instance_id, subframe, slot, value, variant=variant)

        return success

    def delete_subframe(self, frame, instance_id, subframe, variant="main"):
        schema = self.get_subframe_schema(frame, subframe)
        mode = schema["mode"]
        if mode == "single":
            variant = "main"
        elif mode != "multiple":
            return False

        success = self._valkey.jsondelete(frame, f"{instance_id}:{subframe}:{variant}", "$")
        self._valkey.srem(f"{frame}:{instance_id}:{subframe}", "variants", variant)

        slot_keys = self._valkey.scan_keys(f"{frame}:{instance_id}:{subframe}:*:{variant}")
        for full_key in slot_keys:
            namespace, subkey = full_key.split(":", 1)
            self._valkey.delete(namespace, subkey)

        if not self.get_variants(frame, instance_id, subframe):
            self._valkey.delete(f"{frame}:{instance_id}:{subframe}", "variants")

        return success

    def read_subframe(self, frame, instance_id, subframe, path):
        namespace = f"{instance_id}:{subframe}"
        json_path = "$." + path

        result_dict = {}
        variants = self.get_variants(frame, instance_id, subframe)

        for variant in variants:
            result = self._valkey.jsonget(frame, f"{namespace}:{variant}", json_path)
            result_dict[variant] = result

        return result_dict

    def set_slot(self, frame, instance_id, subframe, slot, value, path="$", variant="main"):
        self._valkey.sadd(frame, "instances", instance_id)
        self._valkey.sadd(f"{frame}:{instance_id}:{subframe}", "variants", variant)

        key = f"{instance_id}:{subframe}:{slot}:{variant}"
        success = self._valkey.jsonset(frame, key, path, value)
        self._refresh_ttl(frame, instance_id, subframe, slot, variant)
        return success

    def read_slot_variants(self, frame, instance_id, subframe, slot):
        result_dict = {}
        variants = self.get_variants(frame, instance_id, subframe)

        for variant in variants:
            key = f"{instance_id}:{subframe}:{slot}:{variant}"
            result = self._valkey.jsonget(frame, key, "$")
            result_dict[variant] = result

        return result_dict

    def read_slot(self, frame, instance_id, subframe, slot):
        variants = self.get_variants(frame, instance_id, subframe)
        if not variants:
            self.set_slot(frame, instance_id, subframe, slot, None, variant="main")
            return None

        if "main" not in variants:
            chosen_variant = random.choice(variants)
        else:
            chosen_variant = "main"

        key = f"{instance_id}:{subframe}:{slot}:{chosen_variant}"
        result = self._valkey.jsonget(frame, key, "$")
        return result

    def delete_slot(self, frame, instance_id, subframe, slot, variant="main"):
        key = f"{instance_id}:{subframe}:{slot}:{variant}"
        success = self._valkey.jsondelete(frame, key, "$")
        return success

    def expire_callback(self, key):
        parts = key.split(":")

        if len(parts) != 4:
            return

        frame, instance_id, subframe, variant = parts

        self._valkey.srem(f"{frame}:{instance_id}:{subframe}", "variants", variant)

        slot_keys = self._valkey.scan_keys(f"{frame}:{instance_id}:{subframe}:*:{variant}")
        for full_key in slot_keys:
            namespace, subkey = full_key.split(":", 1)
            self._valkey.delete(namespace, subkey)

        remaining_variants = self.get_variants(frame, instance_id, subframe)

        if not remaining_variants:
            self._valkey.delete(f"{frame}:{instance_id}:{subframe}", "variants")

    def set_fluent(self, fluent, instance, value):
        success = self._valkey.hset(f"fluent:{fluent}", instance, str(value))
        return success

    def del_fluent(self, fluent, instance):
        success = self._valkey.hdel(f"fluent:{fluent}", instance)
        return success

    def read_nullary(self, fluent):
        val = self._valkey.hget(f"fluent:{fluent}", "__flag__")
        return bool(val)

    def read_unary(self, fluent, instance):
        val = self._valkey.hget(f"fluent:{fluent}", instance)
        return bool(val)

    def read_binary(self, fluent, instance1, instance2):
        val = self._valkey.hget(f"fluent:{fluent}", f"{instance1}:{instance2}")
        return bool(val)

    def get_all_groundings(self, fluent):
        vals = self._valkey.hgetall(f"fluent:{fluent}")
        return vals

    def init_observations(self, callback):
        self._valkey.init_observations(callback, self.expire_callback)

    def observe(self, key):
        self._valkey.observe(key)

    def unobserve(self, key):
        self._valkey.unobserve(key)

    def stop_observations(self):
        self._valkey.stop_observations()

    def get_memory_usage(self):
        return self._valkey.get_memory_usage()

    def save(self, filename):
        self._valkey.save(filename)

    def drop(self):
        self._valkey.drop()

    def _extract_slots(self, obj):
        if not isinstance(obj, dict):
            return []
        return list(obj.items())

    def _refresh_ttl(self, frame, instance_id, subframe, slot, variant):
        ttl_raw = self._valkey.get(f"schema:{frame}:{subframe}", "ttl")
        ttl = int(ttl_raw) if ttl_raw not in (None, "") else 0
        if ttl:
            self._valkey.expire(frame, f"{instance_id}:{subframe}:{slot}:{variant}", ttl)

    def get_instance_json(self, frame, instance_id):
        result = {}
        instance_keys = self._valkey.scan_keys(f"{frame}:{instance_id}:*")
        for key in instance_keys:
            try:
                if key.endswith(":variants"):
                    continue
                _, path = key.split(":", 1)
                parts = path.split(":")
                if len(parts) != 4:
                    continue
                _, subframe, slot, variant = parts
                if subframe not in result:
                    result[subframe] = {}

                # TODO Use read slot here and always read main variant
                value = self._valkey.jsonget(frame, path, "$")

                result[subframe][slot] = value
            except ValueError:
                continue

        return result
