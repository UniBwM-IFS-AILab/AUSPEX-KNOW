#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor

from auspex_knowledge.fluent_registry import FluentRegistry
from auspex_knowledge.knowledge_collector import KnowledgeCollector
from auspex_knowledge.knowledge_loader import KnowledgeLoader
from auspex_knowledge.knowledge_server import KnowledgeServer


def main():
    rclpy.init(args=None)

    print("running world knowledge main...")
    know_server = KnowledgeServer()
    know_collector = KnowledgeCollector()
    know_loader = KnowledgeLoader()
    fluent_registry = FluentRegistry()

    node_executor = SingleThreadedExecutor()
    node_executor.add_node(know_server)
    node_executor.add_node(know_collector)
    node_executor.add_node(know_loader)
    node_executor.add_node(fluent_registry)

    try:
        node_executor.spin()
    except Exception as e:
        print("interrupt: shutting down... " + str(e))
    finally:
       fluent_registry.destroy_node()
       know_server.destroy_node()
       know_loader.destroy_node()
       know_collector.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
