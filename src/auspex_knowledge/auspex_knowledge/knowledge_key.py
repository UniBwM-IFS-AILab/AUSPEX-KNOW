#!/usr/bin/env python3

class KnowledgeKey:
    DEFAULT_VARIANT = "__main__"

    @staticmethod
    def instances(frame):
        return f"{frame}:__instances__"

    @staticmethod
    def subframes(frame, instance_id):
        return f"{frame}:{instance_id}:__subframes__"

    @staticmethod
    def variants(frame, instance_id, subframe):
        return f"{frame}:{instance_id}:{subframe}:__variants__"

    @staticmethod
    def slots(frame, instance_id, subframe):
        return f"{frame}:{instance_id}:{subframe}:__slots__"

    @staticmethod
    def schema(frame, subframe, prop):
        return f"__schema__:{frame}:{subframe}:{prop}"

    @staticmethod
    def slot(frame, instance_id, subframe, slot, variant):
        return f"{frame}:{instance_id}:{subframe}:{slot}:{variant}"

    @staticmethod
    def slot_history(frame, instance_id, subframe, slot, variant):
        return f"{frame}:{instance_id}:{subframe}:{slot}:{variant}:__history__"
