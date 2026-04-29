#!/usr/bin/env python3
import json
import random

from auspex_knowledge.knowledge_key import KnowledgeKey
from auspex_knowledge.valkey_client import ValkeyClient


class KnowledgeBase():
    def __init__(self, db_ip="127.0.0.1"):
        self._valkey = ValkeyClient(db_ip)

    def create_frame(self, frame, frame_schema):
        for topic_name, topic_config in frame_schema["topics"].items():
            for property_name, value in topic_config.items():
                self._valkey.set(KnowledgeKey.schema(frame, topic_name, property_name), str(value))

    def exists_instance(self, frame, instance_id):
        return instance_id in self.get_instances(frame)

    def get_instances(self, frame):
        return self._valkey.smembers(KnowledgeKey.instances(frame))

    def delete_instance(self, frame, instance_id):
        subframes = self._valkey.smembers(KnowledgeKey.subframes(frame, instance_id))
        for subframe in subframes:
            variants = self._valkey.smembers(KnowledgeKey.variants(frame, instance_id, subframe))
            for variant in variants:
                self.delete_subframe(frame, instance_id, subframe, variant)
        self._valkey.delete(KnowledgeKey.subframes(frame, instance_id))
        self._valkey.srem(KnowledgeKey.instances(frame), instance_id)
        return True

    def get_variants(self, frame, instance_id, subframe):
        return self._valkey.smembers(KnowledgeKey.variants(frame, instance_id, subframe))

    def upsert_subframe(self, frame, instance_id, subframe, item):
        variant = self._get_variant(frame, subframe, item)

        self._valkey.sadd(KnowledgeKey.instances(frame), instance_id)
        self._valkey.sadd(KnowledgeKey.subframes(frame, instance_id), subframe)
        self._valkey.sadd(KnowledgeKey.variants(frame, instance_id, subframe), variant)

        history = self._get_schema_history(frame, subframe)
        stamp = item.get("header", {}).get("stamp", {}) if history > 0 else {}

        slot_items = self._extract_slots(item)
        for slot, value in slot_items:
            self._valkey.sadd(KnowledgeKey.slots(frame, instance_id, subframe), slot)
            self.set_slot(frame, instance_id, subframe, slot, value, variant=variant)
            if history > 0:
                self._write_history(frame, instance_id, subframe, slot, variant, value, stamp)

        return True

    def delete_subframe(self, frame, instance_id, subframe, variant=KnowledgeKey.DEFAULT_VARIANT):
        slots = self._valkey.smembers(KnowledgeKey.slots(frame, instance_id, subframe))
        for slot in slots:
            self._valkey.delete(KnowledgeKey.slot(frame, instance_id, subframe, slot, variant))

        self._valkey.srem(KnowledgeKey.variants(frame, instance_id, subframe), variant)

        if not self.get_variants(frame, instance_id, subframe):
            self._valkey.delete(KnowledgeKey.variants(frame, instance_id, subframe))
            self._valkey.delete(KnowledgeKey.slots(frame, instance_id, subframe))
            self._valkey.srem(KnowledgeKey.subframes(frame, instance_id), subframe)

        return True

    def _register_slot(self, frame, instance_id, subframe, slot, variant):
        self._valkey.sadd(KnowledgeKey.instances(frame), instance_id)
        self._valkey.sadd(KnowledgeKey.subframes(frame, instance_id), subframe)
        self._valkey.sadd(KnowledgeKey.variants(frame, instance_id, subframe), variant)
        self._valkey.sadd(KnowledgeKey.slots(frame, instance_id, subframe), slot)

    def set_slot(self, frame, instance_id, subframe, slot, value, path="$", variant=KnowledgeKey.DEFAULT_VARIANT):
        self._register_slot(frame, instance_id, subframe, slot, variant)

        key = KnowledgeKey.slot(frame, instance_id, subframe, slot, variant)
        old_val = self._valkey.jsonget(key, path)
        success = True
        if old_val != value:
            success = self._valkey.jsonset(key, path, value)

        ttl = self._get_ttl(frame, subframe)
        if ttl:
            self._valkey.expire(key, ttl)

        return success

    def _write_history(self, frame, instance_id, subframe, slot, variant, value, stamp):
        stream_id = self._stamp_to_stream_id(stamp)
        key = KnowledgeKey.slot_history(frame, instance_id, subframe, slot, variant)
        return self._valkey.xadd(key, self._serialize_entry(value), stream_id)

    def read_history(self, frame, instance_id, subframe, slot, variant=KnowledgeKey.DEFAULT_VARIANT, start="-", end="+", count=None):
        key = KnowledgeKey.slot_history(frame, instance_id, subframe, slot, variant)
        entries = self._valkey.xrange(key, start, end, count) or []

        return [
            {
                "stamp": self._stream_id_to_stamp(stream_id),
                "value": self._deserialize_entry(fields).get("value"),
            }
            for stream_id, fields in entries
        ]

    def read_history_at_stamp(self, frame, instance_id, subframe, slot, stamp, variant=KnowledgeKey.DEFAULT_VARIANT):
        key = KnowledgeKey.slot_history(frame, instance_id, subframe, slot, variant)

        stream_id = self._stamp_to_stream_id(stamp)
        if not stream_id or "*" in stream_id:
            return {}

        entries = self._valkey.xrevrange(key, stream_id, "-", count=1)

        if not entries:
            return {}

        stream_id, fields = entries[0]

        return {
            "stamp": self._stream_id_to_stamp(stream_id),
            "value": self._deserialize_entry(fields).get("value"),
        }

    def read_slot_variants(self, frame, instance_id, subframe, slot):
        result_dict = {}
        variants = self.get_variants(frame, instance_id, subframe)

        for variant in variants:
            key = KnowledgeKey.slot(frame, instance_id, subframe, slot, variant)
            result = self._valkey.jsonget(key, "$")
            result_dict[variant] = result

        return result_dict

    def read_slot(self, frame, instance_id, subframe, slot):
        variants = self.get_variants(frame, instance_id, subframe)
        if not variants:
            self.set_slot(frame, instance_id, subframe, slot, None, variant=KnowledgeKey.DEFAULT_VARIANT)
            return None

        if KnowledgeKey.DEFAULT_VARIANT not in variants:
            chosen_variant = random.choice(list(variants))
        else:
            chosen_variant = KnowledgeKey.DEFAULT_VARIANT

        key = KnowledgeKey.slot(frame, instance_id, subframe, slot, chosen_variant)
        result = self._valkey.jsonget(key, "$")
        return result

    def delete_slot(self, frame, instance_id, subframe, slot, variant=KnowledgeKey.DEFAULT_VARIANT):
        key = KnowledgeKey.slot(frame, instance_id, subframe, slot, variant)
        success = self._valkey.delete(key)
        return success

    def get_instance_json(self, frame, instance_id):
        result = {}
        subframes = self._valkey.smembers(KnowledgeKey.subframes(frame, instance_id))

        for subframe in subframes:
            result[subframe] = {}
            slots = self._valkey.smembers(KnowledgeKey.slots(frame, instance_id, subframe))

            for slot in slots:
                key = KnowledgeKey.slot(frame, instance_id, subframe, slot, KnowledgeKey.DEFAULT_VARIANT)
                value = self._valkey.jsonget(key, "$")
                result[subframe][slot] = value

        return result

    def expire_callback(self, key):
        parts = key.split(":")

        if len(parts) != 5:
            return

        self._valkey.delete(key)

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

    def drop(self):
        self._valkey.drop()

    def _get_ttl(self, frame, subframe):
        ttl_raw = self._valkey.get(KnowledgeKey.schema(frame, subframe, "ttl"))
        ttl = int(ttl_raw) if ttl_raw not in (None, "") else 0
        return ttl

    def _get_variant(self, frame, subframe, item=None):
        mode = self._valkey.get(KnowledgeKey.schema(frame, subframe, "mode")) or "single"
        if mode == "multiple" and item is not None:
            variant_key = self._valkey.get(KnowledgeKey.schema(frame, subframe, "variant_key")) or None
            if not variant_key:
                return KnowledgeKey.DEFAULT_VARIANT
            return item.get(variant_key) or KnowledgeKey.DEFAULT_VARIANT
        return KnowledgeKey.DEFAULT_VARIANT

    def _get_schema_history(self, frame, subframe):
        history_str = self._valkey.get(KnowledgeKey.schema(frame, subframe, "history"))
        history = int(history_str) if history_str not in (None, "") else 0
        return history

    def _extract_slots(self, obj):
        if not isinstance(obj, dict):
            return []
        return list(obj.items())

    def _stamp_to_stream_id(self, stamp):
        sec = stamp.get("sec")
        nanosec = stamp.get("nanosec")

        if sec is None or nanosec is None:
            return "*"

        millis = sec * 1000 + nanosec // 1_000_000
        sequence = nanosec % 1_000_000

        return f"{millis}-{sequence}"

    def _stream_id_to_stamp(self, stream_id):
        millis_str, sequence_str = stream_id.split("-", 1)

        millis = int(millis_str)
        sequence = int(sequence_str)

        return {
            "sec": millis // 1000,
            "nanosec": (millis % 1000) * 1_000_000 + sequence,
        }

    def _serialize_entry(self, value):
        try:
            return {"value": json.dumps(value)}
        except TypeError:
            return {"value": str(value)}

    def _deserialize_entry(self, fields):
        raw = fields.get("value")

        try:
            return {"value": json.loads(raw)}
        except (TypeError, json.JSONDecodeError):
            return {"value": raw}
