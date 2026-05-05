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
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from af_msgs.msg import MissionCommand, MissionStatus


_EXAMPLES_TABLE = """
┌──────────────────┬────────────────────────────────────────────────┐
│ Command          │ Example phrases                                │
├──────────────────┼────────────────────────────────────────────────┤
│ find_object      │ "find the suitcase"                            │
│                  │ "search for a bottle"                          │
│                  │ "look for my backpack"                         │
├──────────────────┼────────────────────────────────────────────────┤
│ drive_for        │ "move forward for 3 seconds"                   │
│                  │ "go left for 2 seconds"                        │
│                  │ "drive backward slowly for 5 seconds"          │
│                  │ "turn left 90 degrees"                         │
│                  │ "rotate right for 2 seconds"                   │
├──────────────────┼────────────────────────────────────────────────┤
│ scan_area        │ "explore for 30 seconds"                       │
│                  │ "scan the area for 5 metres"                   │
│                  │ "map the entire room"                          │
├──────────────────┼────────────────────────────────────────────────┤
│ patrol           │ "patrol the door and far wall"                 │
│                  │ "visit corner a then corner b"                 │
├──────────────────┼────────────────────────────────────────────────┤
│ set_speed        │ "set speed to 0.1"                             │
│                  │ "slow down"                                    │
│                  │ "go faster"                                    │
├──────────────────┼────────────────────────────────────────────────┤
│ stop             │ "stop"                                         │
│                  │ "halt"                                         │
│                  │ "freeze"                                       │
└──────────────────┴────────────────────────────────────────────────┘
""".strip()


_STATE_LABELS = {
    'idle':             'Idle — waiting for command',
    'capture_home':     'Capturing home position...',
    'exploring':        'Exploring',
    'target_confirmed': 'Target confirmed — stopping exploration',
    'return_home':      'Navigating home...',
    'done':             'Mission complete',
    'failed':           'Mission failed',
}

_ACTIVE_STATES = {'capture_home', 'exploring', 'target_confirmed', 'return_home'}


class NlpCli(Node):
    def __init__(self):
        super().__init__('nlp_cli')
        self._pub = self.create_publisher(String, '/nlp/text_input', 10)
        self.create_subscription(String, '/nlp/response', self._on_response, 10)
        self.create_subscription(String, '/nlp/clarification_needed', self._on_clarify, 10)
        self.create_subscription(MissionCommand, '/mission/command', self._on_cmd, 10)
        self.create_subscription(MissionStatus, '/mission/status', self._on_status, 10)
        self._last_state = ''
        self._last_explore_print = 0.0
        self._in_find_mission = False

    def _on_response(self, msg):
        if msg.data:
            print(f'\n  [LLM] {msg.data}', flush=True)

    def _on_clarify(self, msg):
        print(f'\n  [???] {msg.data}', flush=True)

    def _on_cmd(self, msg):
        print(f'  [CMD] {msg.command_type} → {msg.parameters_json}', flush=True)

    def _on_status(self, msg: MissionStatus):
        state = msg.state

        if state == 'capture_home':
            self._in_find_mission = True
        elif state in ('idle',):
            self._in_find_mission = False

        if not self._in_find_mission:
            return

        changed = state != self._last_state
        if changed:
            self._last_state = state
            label = _STATE_LABELS.get(state, state)

            if state == 'failed':
                reason = msg.error_message or 'unknown'
                print(f'\n  [STATUS] {label}: {reason}', flush=True)
                self._in_find_mission = False
            elif state == 'done':
                print(f'\n  [STATUS] {label} — {msg.elapsed_s:.0f}s, {msg.distance_travelled_m:.1f}m', flush=True)
                self._in_find_mission = False
            elif state in _ACTIVE_STATES:
                print(f'\n  [STATUS] {label}', flush=True)
            self._last_explore_print = 0.0

        if state == 'exploring':
            now = time.monotonic()
            if now - self._last_explore_print >= 5.0:
                self._last_explore_print = now
                print(f'  [STATUS] Exploring — {msg.elapsed_s:.0f}s elapsed, '
                      f'{msg.distance_travelled_m:.1f}m travelled', flush=True)

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
    print('Type /Examples to see full list of available commands\n')

    try:
        while True:
            text = input('> ').strip()
            if not text:
                continue
            if text.lower() in ('/examples', '/example', '/help'):
                print(_EXAMPLES_TABLE)
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
