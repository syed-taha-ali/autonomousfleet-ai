#!/usr/bin/env python3
"""Interactive CLI for sending natural-language commands to the robot.

Usage:
  ros2 run af_nlp cli

Type commands at the prompt. They are published to /nlp/text_input for
nlp_command_node to translate and dispatch. Responses and clarifications
are printed as they arrive.
"""
import readline
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from af_msgs.msg import MissionCommand


class NlpCli(Node):
    def __init__(self):
        super().__init__('nlp_cli')
        self._pub = self.create_publisher(String, '/nlp/text_input', 10)
        self.create_subscription(String, '/nlp/response', self._on_response, 10)
        self.create_subscription(String, '/nlp/clarification_needed', self._on_clarify, 10)
        self.create_subscription(MissionCommand, '/mission/command', self._on_cmd, 10)

    def _on_response(self, msg):
        if msg.data:
            print(f'\n  [LLM] {msg.data}')

    def _on_clarify(self, msg):
        print(f'\n  [???] {msg.data}')

    def _on_cmd(self, msg):
        print(f'  [CMD] {msg.command_type} → {msg.parameters_json}')

    def send(self, text):
        m = String()
        m.data = text
        self._pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = NlpCli()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('AutonomousFleet NLP CLI — type commands, Ctrl+C to exit')
    print('Examples: "find the suitcase", "go to the door", "stop"\n')

    try:
        while True:
            text = input('> ').strip()
            if not text:
                continue
            node.send(text)
    except (KeyboardInterrupt, EOFError):
        print('\nExiting.')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
