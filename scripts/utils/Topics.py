#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .Controller import Controller, Axis, Button


class Topics:
    def __init__(self, node: Node, mapping: dict):
        self._topics = dict()
        if mapping:
            for key, value in mapping.items():
                if value['type'] == 'Twist':
                    self._topics[key] = TwistTopic(node, value)

    def publish(self, controller: Controller):
        for _, topic in self._topics.items():
            topic.publish(controller)

    def __getitem__(self, item):
        return self._buttons

    def __iter__(self):
        return self._buttons.__iter__()

    def __str__(self):
        return '{' + ', '.join(['%s.: %s.' % (key, value.state) for key, value in self._buttons.items()]) + '}'


class TwistTopic:
    def __init__(self, node: Node, params: dict):
        self._node = node
        self.topic = params['topic']

        self.x = params['data']['linear']['x']
        self.y = params['data']['linear']['y']
        self.z = params['data']['linear']['z']

        self.roll = params['data']['angular']['x']
        self.pitch = params['data']['angular']['y']
        self.yaw = params['data']['angular']['z']

        self._publisher = node.create_publisher(Twist, self.topic)

    def publish(self, controller: Controller):
        msg = Twist()
        msg.linear.x = self._convert_input(self.x, controller)
        msg.linear.y = self._convert_input(self.y, controller)
        msg.linear.z = self._convert_input(self.z, controller)
        msg.angular.x = self._convert_input(self.roll, controller)
        msg.angular.y = self._convert_input(self.pitch, controller)
        msg.angular.z = self._convert_input(self.yaw, controller)
        self._publisher.publish(msg)

    def _convert_input(self, param, controller: Controller):
        param_type = type(param)
        if param_type == str:
            self._node.get_logger().fatal(str(type(param)))
            return float(controller[param].state)
        elif param_type == int or param_type == float:
            return float(param)
        elif param is None:
            return float(0)
        else:
            self._node.get_logger().warn('Parameter "%s." should be numeric or None.' % str(param))
            return float(0)
