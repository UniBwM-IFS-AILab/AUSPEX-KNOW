#!/usr/bin/env python3
from rclpy.node import Node

from auspex_msgs.srv import RegisterNullaryFluent, RegisterUnaryFluent, RegisterBinaryFluent
from auspex_msgs.srv import UnregisterNullaryFluent, UnregisterUnaryFluent, UnregisterBinaryFluent
from auspex_msgs.srv import ReadNullaryFluent, ReadUnaryFluent, ReadBinaryFluent
from auspex_msgs.msg import FluentChange

from auspex_knowledge.knowledge_base import KnowledgeBase
from auspex_knowledge.eval_func import compile_lambda, aggregate_bool, eval_dict

class FluentRegistry(Node):
    def __init__(self):
        super().__init__("fluent_registry")
        self._know_base = KnowledgeBase()

        self._nullary_fluents = {}
        self._unary_fluents = {}
        self._binary_fluents = {}

        self._observed_fluents = {}
        self._know_base.init_observations(self.observation_callback)

        self.register_nullary_fluent_srv = self.create_service(RegisterNullaryFluent, "register_nullary_fluent", self.handle_register_nullary_fluent)
        self.register_unary_fluent_srv = self.create_service(RegisterUnaryFluent, "register_unary_fluent", self.handle_register_unary_fluent)
        self.register_binary_fluent_srv = self.create_service(RegisterBinaryFluent, "register_binary_fluent", self.handle_register_binary_fluent)
        self.unregister_nullary_fluent_srv = self.create_service(UnregisterNullaryFluent, "unregister_nullary_fluent", self.handle_unregister_nullary_fluent)
        self.unregister_unary_fluent_srv = self.create_service(UnregisterUnaryFluent, "unregister_unary_fluent", self.handle_unregister_unary_fluent)
        self.unregister_binary_fluent_srv = self.create_service(UnregisterBinaryFluent, "unregister_binary_fluent", self.handle_unregister_binary_fluent)
        self.read_nullary_fluent_srv = self.create_service(ReadNullaryFluent, "read_nullary_fluent", self.handle_read_nullary_fluent)
        self.read_unary_fluent_srv = self.create_service(ReadUnaryFluent, "read_unary_fluent", self.handle_read_unary_fluent)
        self.read_binary_fluent_srv = self.create_service(ReadBinaryFluent, "read_binary_fluent", self.handle_read_binary_fluent)

        self._change_publisher = self.create_publisher(FluentChange, "/fluent_change", 10)

    def destroy_node(self):
        self._know_base.stop_observations()
        return super().destroy_node()

    def handle_register_nullary_fluent(self, request, response):
        response.success = self.register_nullary(request.fluent, request.frame, request.subframe, request.slot, request.lambda_str)
        return response

    def handle_register_unary_fluent(self, request, response):
        response.success = self.register_unary(request.fluent, request.frame, request.subframe, request.slot, request.lambda_str)
        return response

    def handle_register_binary_fluent(self, request, response):
        response.success = self.register_binary(request.fluent, request.frame1, request.subframe1, request.slot1, request.frame2, request.subframe2, request.slot2, request.lambda_str)
        return response

    def handle_unregister_nullary_fluent(self, request, response):
        try:
            del self._nullary_fluents[request.fluent]
            response.success = True
        except KeyError:
            print(f"no fluent {request.fluent} is registered.")
            response.success = False
        return response

    def handle_unregister_unary_fluent(self, request, response):
        try:
            del self._unary_fluents[request.fluent]
            response.success = True
        except KeyError:
            print(f"no fluent {request.fluent} is registered.")
            response.success = False
        return response

    def handle_unregister_binary_fluent(self, request, response):
        try:
            del self._binary_fluents[request.fluent]
            response.success = True
        except KeyError:
            print(f"no fluent {request.fluent} is registered.")
            response.success = False
        return response

    def handle_read_nullary_fluent(self, request, response):
        response.val = str(self.read_nullary(request.fluent))
        return response

    def handle_read_unary_fluent(self, request, response):
        response.val = str(self.read_unary(request.fluent, request.instance_id1))
        return response

    def handle_read_binary_fluent(self, request, response):
        response.val = str(self.read_binary(request.fluent, request.instance_id1, request.instance_id2))
        return response

    def _match_key_pattern(self, concrete_key, wildcard_key):
        parts_concrete = concrete_key.split(":")
        parts_wildcard = wildcard_key.split(":")

        if len(parts_concrete) != len(parts_wildcard):
            return None

        for part_concrete, part_wildcard in zip(parts_concrete, parts_wildcard):
            if part_wildcard != "*" and part_concrete != part_wildcard:
                return None
        return parts_concrete[1] if len(parts_concrete) > 1 else None

    def observation_callback(self, key):
        for fluent, key_spaces in list(self._observed_fluents.items()):
            matched_index = None
            matched_instance_id = None

            for i, key_space in enumerate(key_spaces):
                instance_id = self._match_key_pattern(key, key_space)
                if instance_id is not None:
                    matched_index = i
                    matched_instance_id = instance_id
                    break

            if matched_instance_id is None:
                continue

            if fluent in self._nullary_fluents:
                old_val = self.read_nullary(fluent)
                self.evaluate_nullary(fluent)
                new_val = self.read_nullary(fluent)

                if old_val != new_val:
                    msg = FluentChange()
                    msg.fluent = fluent
                    msg.objects = []
                    msg.new_value = str(new_val)
                    self._change_publisher.publish(msg)

            elif fluent in self._unary_fluents:
                old_val = self.read_unary(fluent, matched_instance_id)
                self.evaluate_unary(fluent, matched_instance_id)
                new_val = self.read_unary(fluent, matched_instance_id)

                if old_val != new_val:
                    msg = FluentChange()
                    msg.fluent = fluent
                    msg.objects = [matched_instance_id]
                    msg.new_value = str(new_val)
                    self._change_publisher.publish(msg)

            elif fluent in self._binary_fluents:
                frame1 = self._binary_fluents[fluent]["frame1"]
                frame2 = self._binary_fluents[fluent]["frame2"]

                if matched_index == 0:
                    instance_ids2 = self._know_base.get_instances(frame2)
                    for instance_id2 in instance_ids2:
                        old_val = self.read_binary(fluent, matched_instance_id, instance_id2)
                        self.evaluate_binary(fluent, matched_instance_id, instance_id2)
                        new_val = self.read_binary(fluent, matched_instance_id, instance_id2)

                        if old_val != new_val:
                            msg = FluentChange()
                            msg.fluent = fluent
                            msg.objects = [matched_instance_id, instance_id2]
                            msg.new_value = str(new_val)
                            self._change_publisher.publish(msg)

                elif matched_index == 1:
                    instance_ids1 = self._know_base.get_instances(frame1)
                    for instance_id1 in instance_ids1:
                        old_val = self.read_binary(fluent, instance_id1, matched_instance_id)
                        self.evaluate_binary(fluent, instance_id1, matched_instance_id)
                        new_val = self.read_binary(fluent, instance_id1, matched_instance_id)

                        if old_val != new_val:
                            msg = FluentChange()
                            msg.fluent = fluent
                            msg.objects = [instance_id1, matched_instance_id]
                            msg.new_value = str(new_val)
                            self._change_publisher.publish(msg)

    def observe(self, fluent):
        if fluent in self._nullary_fluents:
            frame = self._nullary_fluents[fluent]["frame"]
            subframe = self._nullary_fluents[fluent]["subframe"]
            slot = self._nullary_fluents[fluent]["slot"]
            key_space = f"{frame}:*:{subframe}:{slot}:*"
            self._know_base.observe(key_space)
            self._observed_fluents[fluent] = [key_space]

        elif fluent in self._unary_fluents:
            frame = self._unary_fluents[fluent]["frame"]
            subframe = self._unary_fluents[fluent]["subframe"]
            slot = self._unary_fluents[fluent]["slot"]
            key_space = f"{frame}:*:{subframe}:{slot}:*"
            self._know_base.observe(key_space)
            self._observed_fluents[fluent] = [key_space]

        elif fluent in self._binary_fluents:
            frame1 = self._binary_fluents[fluent]["frame1"]
            subframe1 = self._binary_fluents[fluent]["subframe1"]
            slot1 = self._binary_fluents[fluent]["slot1"]
            key_space1 = f"{frame1}:*:{subframe1}:{slot1}:*"

            frame2 = self._binary_fluents[fluent]["frame2"]
            subframe2 = self._binary_fluents[fluent]["subframe2"]
            slot2 = self._binary_fluents[fluent]["slot2"]
            key_space2 = f"{frame2}:*:{subframe2}:{slot2}:*"

            self._know_base.observe(key_space1)
            self._know_base.observe(key_space2)

            self._observed_fluents[fluent] = [key_space1, key_space2]

    def unobserve(self, fluent):
        if fluent not in self._observed_fluents:
            return

        key_spaces = self._observed_fluents.pop(fluent)

        remaining_key_spaces = set()
        for other_key_spaces in self._observed_fluents.values():
            remaining_key_spaces.update(other_key_spaces)

        for key_space in key_spaces:
            if key_space not in remaining_key_spaces:
                self._know_base.unobserve(key_space)

    def register_nullary(self, fluent, frame, subframe, slot, lambda_str, aggregation="any"):
        eval_lambda = compile_lambda(lambda_str, 1)
        if not callable(eval_lambda):
            print(f"error: lambda not callable: {lambda_str}")
            return False
        self._nullary_fluents[fluent] = {"frame": frame, "subframe": subframe, "slot": slot, "eval_lambda": eval_lambda, "aggregation": aggregation}
        return True

    def register_unary(self, fluent, frame, subframe, slot, lambda_str, aggregation="any"):
        eval_lambda = compile_lambda(lambda_str, 1)
        if not callable(eval_lambda):
            print(f"error: lambda not callable: {lambda_str}")
            return False
        self._unary_fluents[fluent] = {"frame": frame, "subframe": subframe, "slot": slot, "eval_lambda": eval_lambda, "aggregation": aggregation}
        return True

    def register_binary(self, fluent, frame1, subframe1, slot1, frame2, subframe2, slot2, lambda_str, aggregation="any"):
        eval_lambda = compile_lambda(lambda_str, 2)
        if not callable(eval_lambda):
            print(f"error: lambda not callable: {lambda_str}")
            return False
        self._binary_fluents[fluent] = {"frame1": frame1, "subframe1": subframe1, "slot1": slot1, "frame2": frame2, "subframe2": subframe2, "slot2": slot2, "eval_lambda": eval_lambda, "aggregation": aggregation}
        return True

    def evaluate_nullary(self, fluent):
        frame = self._nullary_fluents[fluent]["frame"]
        subframe = self._nullary_fluents[fluent]["subframe"]
        slot = self._nullary_fluents[fluent]["slot"]
        eval_lambda = self._nullary_fluents[fluent]["eval_lambda"]
        aggregation = self._nullary_fluents[fluent]["aggregation"]

        instance_ids = self._know_base.get_instances(frame)
        results = []

        for instance_id in instance_ids:
            slot_value = self._know_base.read_slot_variants(frame, instance_id, subframe, slot)
            if not slot_value:
                results.append(False)
                continue
            results.append(bool(eval_dict(slot_value, eval_lambda, aggregation)))
        eval_result = aggregate_bool(results, aggregation)
        if eval_result:
            self._know_base.set_fluent(fluent, "__flag__", True)
        else:
            self._know_base.del_fluent(fluent, "__flag__")
        return eval_result

    def evaluate_unary(self, fluent, instance_id):
        frame = self._unary_fluents[fluent]["frame"]
        subframe = self._unary_fluents[fluent]["subframe"]
        slot =  self._unary_fluents[fluent]["slot"]
        eval_lambda = self._unary_fluents[fluent]["eval_lambda"]
        aggregation = self._unary_fluents[fluent]["aggregation"]

        slot_value = self._know_base.read_slot_variants(frame, instance_id, subframe, slot)

        if not slot_value:
            self._know_base.del_fluent(fluent, instance_id)
            return False
        eval_result = bool(eval_dict(slot_value, eval_lambda, aggregation))

        if eval_result:
            self._know_base.set_fluent(fluent, instance_id, eval_result)
        else:
            self._know_base.del_fluent(fluent, instance_id)
        return eval_result

    def evaluate_binary(self, fluent, instance_id1, instance_id2):
        grounding_id = f"{instance_id1}:{instance_id2}"

        frame1 = self._binary_fluents[fluent]["frame1"]
        subframe1 = self._binary_fluents[fluent]["subframe1"]
        slot1 = self._binary_fluents[fluent]["slot1"]

        frame2 = self._binary_fluents[fluent]["frame2"]
        subframe2 = self._binary_fluents[fluent]["subframe2"]
        slot2 = self._binary_fluents[fluent]["slot2"]

        eval_lambda = self._binary_fluents[fluent]["eval_lambda"]
        aggregation = self._binary_fluents[fluent]["aggregation"]

        slot_value1 = self._know_base.read_slot_variants(frame1, instance_id1, subframe1, slot1)
        if not slot_value1:
            self._know_base.del_fluent(fluent, grounding_id)
            return False

        slot_value2 = self._know_base.read_slot_variants(frame2, instance_id2, subframe2, slot2)
        if not slot_value2:
            self._know_base.del_fluent(fluent, grounding_id)
            return False

        results = []

        values1 = [v for v in slot_value1.values() if v is not None]
        values2 = [v for v in slot_value2.values() if v is not None]

        for v1 in values1:
            for v2 in values2:
                results.append(bool(eval_lambda(v1, v2)))

        eval_result = aggregate_bool(results, aggregation) if results else False

        if eval_result:
            self._know_base.set_fluent(fluent, grounding_id, eval_result)
        else:
            self._know_base.del_fluent(fluent, grounding_id)

        return eval_result

    def ground_nullary(self, fluent):
        self.evaluate_nullary(fluent)

    def ground_unary(self, fluent):
        frame = self._unary_fluents[fluent]["frame"]
        instance_ids = self._know_base.get_instances(frame)

        for instance_id in instance_ids:
            self.evaluate_unary(fluent, instance_id)

    def ground_binary(self, fluent):
        frame1 = self._binary_fluents[fluent]["frame1"]
        frame2 = self._binary_fluents[fluent]["frame2"]

        instance_ids1 = self._know_base.get_instances(frame1)
        instance_ids2 = self._know_base.get_instances(frame2)

        for instance_id1 in instance_ids1:
            for instance_id2 in instance_ids2:
                self.evaluate_binary(fluent, instance_id1, instance_id2)

    def snapshot(self):
        for fluent in self._nullary_fluents.keys():
            self.ground_nullary(fluent)

        for fluent in self._unary_fluents.keys():
            self.ground_unary(fluent)

        for fluent in self._binary_fluents.keys():
            self.ground_binary(fluent)

        snapshot = {
            "nullary": self.get_groundings_nullary(),
            "unary": {},
            "binary": {}
        }

        for fluent in self._unary_fluents.keys():
            vals = self.get_groundings_unary(fluent)
            if vals:
                snapshot["unary"][fluent] = vals

        for fluent in self._binary_fluents.keys():
            vals = self.get_groundings_binary(fluent)
            if vals:
                snapshot["binary"][fluent] = vals

        return snapshot

    def read_nullary(self, fluent):
        return self._know_base.read_nullary(fluent)

    def read_unary(self, fluent, instance_id):
        return self._know_base.read_unary(fluent, instance_id)

    def read_binary(self, fluent, instance_id1, instance_id2):
        return self._know_base.read_binary(fluent, instance_id1, instance_id2)

    def get_groundings_nullary(self):
        result = {}

        for fluent in self._nullary_fluents.keys():
            if self.read_nullary(fluent):
                result[fluent] = True

        return result

    def get_groundings_unary(self, fluent):
        vals = self._know_base.get_all_groundings(fluent)

        if not vals:
            return []

        return list(vals.keys())

    def get_groundings_binary(self, fluent):
        vals = self._know_base.get_all_groundings(fluent)

        if not vals:
            return []

        pairs = []
        for key in vals.keys():
            obj1, obj2 = key.split(":", 1)
            pairs.append((obj1, obj2))

        return pairs
