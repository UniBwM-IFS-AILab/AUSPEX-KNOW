import rclpy
from rclpy.executors import SingleThreadedExecutor
from .world_knowledge_base import WorldKnowledgeBase
from .world_knowledge_server import WorldKnowledgeServer
from .knowledge_collector import KnowledgeCollector


def main():
    rclpy.init(args=None)

    print('running world knowledge main...')
    wkb = WorldKnowledgeBase()
    wks = WorldKnowledgeServer(wkb)
    kc = KnowledgeCollector(wkb)

    ste = SingleThreadedExecutor()
    ste.add_node(wkb)
    ste.add_node(wks)
    ste.add_node(kc)

    try:
        ste.spin()
    except Exception as e:
        print('interrupt: shutting down... ' + str(e))
    finally:
        wkb.destroy_node()
        wks.destroy_node()
        kc.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
