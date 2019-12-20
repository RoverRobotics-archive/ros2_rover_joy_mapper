#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import yaml
from Controller import Controller, Axis, Button
from Topics import Topics, TwistTopic

class Mapper(Node):
    def __init__(self, joy_topic, controller=None, topics=None, name="joy_mapper_node"):
        super().__init__(name)
        self._name = name
        self._joy_topic = joy_topic
        self._publish_topics = []

        self._joy_subscriber = self.create_subscription(Joy, joy_topic, self._joy_callback, 10)
        self._controller = self.configure_controller_mapping(controller) if controller else None
        self._topics = self.register_topics(topics) if topics else None

    def _joy_callback(self, msg: Joy):
        if not self._controller:
            self.get_logger().fatal('Axis and Button mappings must be defined.')
            self.shutdown()
        elif not self._topics:
            self.get_logger().fatal('No topics specified')
            self.shutdown()
        self._controller.update_states(**{'axes': msg.axes, 'buttons': msg.buttons})

        self._topics.publish(self._controller)

    def configure_controller_mapping(self, button_mappings):
        self._controller = Controller(button_mappings)

    def register_topics(self, topics: dict):
        self._topics = Topics(self, topics)

    def update_topics(self):
        for topic in self._topics:
            topic.publish_topic(self.state)


def open_yaml(file_name):
    with open(file_name) as stream:
        return yaml.safe_load(stream)

def main(args=None):
    rclpy.init(args=args)
    controller = open_yaml('/home/ros/demo_ws/src/openrover/openrover_joy_mapper_throttle/config/controller.yaml')
    topics = open_yaml('/home/ros/demo_ws/src/openrover/openrover_joy_mapper_throttle/config/topics.yaml')
    mapper = Mapper('/joy')
    mapper.configure_controller_mapping(controller)
    mapper.register_topics(topics)
    rclpy.spin(mapper)

if __name__ == '__main__':
    main()
