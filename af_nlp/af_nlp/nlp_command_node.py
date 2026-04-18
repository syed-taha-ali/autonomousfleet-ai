#!/usr/bin/env python3
"""NLP command node: translates natural language to ROS 2 mission commands.

Accepts text on /nlp/text_input, sends it to a local Ollama LLM with a
tool-calling schema that mirrors the robot's action servers, and publishes
the resulting MissionCommand on /mission/command for the mission_manager
to dispatch.

Runs on the Dev PC (not the Pi). Requires Ollama running locally.
"""
import json
import os
import re
import time
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from af_msgs.msg import MissionCommand

_TOOL_SCHEMA = [
    {
        'type': 'function',
        'function': {
            'name': 'find_object',
            'description': (
                'Search the environment for a specific object. The robot will '
                'explore autonomously and detect the object using its camera.'
            ),
            'parameters': {
                'type': 'object',
                'properties': {
                    'target_class': {
                        'type': 'string',
                        'description': (
                            'COCO object class name to search for (e.g. '
                            '"suitcase", "bottle", "cup", "chair", "person", '
                            '"book", "cell phone", "laptop", "backpack").'
                        ),
                    },
                    'confidence_min': {
                        'type': 'number',
                        'description': 'Minimum detection confidence (0.0-1.0). Default 0.5.',
                    },
                    'max_duration_s': {
                        'type': 'number',
                        'description': 'Maximum search time in seconds. Default 300.',
                    },
                },
                'required': ['target_class'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'patrol',
            'description': (
                'Send the robot on a patrol route through stored room locations, '
                'visiting each in order. Rooms must be pre-configured in the '
                'rooms file.'
            ),
            'parameters': {
                'type': 'object',
                'properties': {
                    'room_ids': {
                        'type': 'array',
                        'items': {'type': 'string'},
                        'description': 'List of room IDs to visit in order.',
                    },
                },
                'required': ['room_ids'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'drive_for',
            'description': (
                'Drive or rotate the robot. For translation: specify a direction '
                'and duration. For rotation: use rotate_left or rotate_right '
                'with either a duration or an angle in degrees.'
            ),
            'parameters': {
                'type': 'object',
                'properties': {
                    'direction': {
                        'type': 'string',
                        'enum': [
                            'forward', 'backward', 'left', 'right',
                            'forward_left', 'forward_right',
                            'backward_left', 'backward_right',
                            'rotate_left', 'rotate_right',
                        ],
                        'description': 'Direction to drive or rotate.',
                    },
                    'speed': {
                        'type': 'number',
                        'description': 'Speed in m/s (0.0-0.2) for translation, or rad/s (0.0-1.0) for rotation. Default 0.15.',
                    },
                    'duration_s': {
                        'type': 'number',
                        'description': 'How long to drive/rotate in seconds. Required unless angle_deg is set.',
                    },
                    'angle_deg': {
                        'type': 'number',
                        'description': 'Rotation angle in degrees. Only for rotate_left/rotate_right. Overrides duration_s.',
                    },
                },
                'required': ['direction'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'stop',
            'description': (
                'Immediately stop the robot and cancel all active navigation '
                'goals. Use for emergency stops or when the user wants the '
                'robot to halt.'
            ),
            'parameters': {
                'type': 'object',
                'properties': {},
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'set_speed',
            'description': (
                'Set the maximum speed limits for the robot. Affects all '
                'movement commands including navigation and open-loop driving. '
                'Linear speed in m/s (max 0.3), angular speed in rad/s (max 1.0).'
            ),
            'parameters': {
                'type': 'object',
                'properties': {
                    'max_linear': {
                        'type': 'number',
                        'description': 'Maximum linear speed in m/s (0.0-0.3).',
                    },
                    'max_angular': {
                        'type': 'number',
                        'description': 'Maximum angular speed in rad/s (0.0-1.0).',
                    },
                },
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'scan_area',
            'description': (
                'Explore and map the surrounding area. The robot will '
                'autonomously navigate around the room. Stops in place when '
                'done. Specify a time limit, distance limit, or use "full" '
                'mode to scan until no unexplored areas remain.'
            ),
            'parameters': {
                'type': 'object',
                'properties': {
                    'max_time_s': {
                        'type': 'number',
                        'description': 'Maximum exploration time in seconds.',
                    },
                    'max_distance_m': {
                        'type': 'number',
                        'description': 'Maximum exploration distance in metres.',
                    },
                    'mode': {
                        'type': 'string',
                        'enum': ['timed', 'distance', 'full'],
                        'description': 'Exploration mode. "timed" uses max_time_s, "distance" uses max_distance_m, "full" scans until complete. Default "timed".',
                    },
                },
            },
        },
    },
]

_COCO_CLASSES = (
    'person, bicycle, car, motorcycle, airplane, bus, train, truck, boat, '
    'traffic light, fire hydrant, stop sign, parking meter, bench, bird, cat, '
    'dog, horse, sheep, cow, elephant, bear, zebra, giraffe, backpack, '
    'umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports ball, '
    'kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket, '
    'bottle, wine glass, cup, fork, knife, spoon, bowl, banana, apple, '
    'sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake, chair, '
    'couch, potted plant, bed, dining table, toilet, tv, laptop, mouse, '
    'remote, keyboard, cell phone, microwave, oven, toaster, sink, '
    'refrigerator, book, clock, vase, scissors, teddy bear, hair drier, '
    'toothbrush'
)


def _build_system_prompt(rooms: dict) -> str:
    room_lines = '\n'.join(
        f'  - {name}: ({p["x"]:.2f}, {p["y"]:.2f})'
        for name, p in rooms.items()
    ) if rooms else '  (none configured)'
    return (
        'You are the command interface for an autonomous ground robot. '
        'The user gives natural-language instructions and you MUST translate '
        'them into exactly one tool call. ALWAYS respond with a tool call, '
        'never with plain text alone.\n\n'
        'Rules:\n'
        '- You MUST call exactly one tool for every user message. No exceptions.\n'
        '- If the request is ambiguous, pick the most reasonable interpretation '
        'and call the tool.\n'
        '- If the request seems dangerous, call stop().\n'
        '- For object search tasks, use find_object with the closest COCO class.\n'
        '- "stop", "halt", "freeze", "don\'t move", "emergency" → call stop().\n'
        '- "explore", "look around", "scan", "map the area" → call scan_area().\n'
        '- "patrol", "visit", "go around" with room names → call patrol().\n'
        '- "turn left/right", "rotate" → call drive_for() with rotate_left/rotate_right.\n'
        '- "slow down", "speed up", "set speed" → call set_speed().\n\n'
        f'Available rooms for patrol:\n{room_lines}\n\n'
        f'Detectable object classes (COCO): {_COCO_CLASSES}\n'
    )


class NlpCommandNode(Node):
    def __init__(self):
        super().__init__('nlp_command')

        self.declare_parameter('ollama_url', 'http://localhost:11434')
        self.declare_parameter('model', 'qwen2.5:7b-instruct-q4_K_M')
        self.declare_parameter('rooms_file', '')
        self.declare_parameter('log_file', '')
        self.declare_parameter('timeout_s', 60.0)

        self._ollama_url = self.get_parameter('ollama_url').value
        self._model = self.get_parameter('model').value
        self._timeout = self.get_parameter('timeout_s').value

        self._rooms = self._load_rooms()

        log_path = self.get_parameter('log_file').value
        self._log_file = open(log_path, 'a') if log_path else None

        self._system_prompt = _build_system_prompt(self._rooms)
        self._processing = False
        self._last_text = ''
        self._last_text_time = 0.0

        self._cmd_pub = self.create_publisher(MissionCommand, '/mission/command', 10)
        self._clarify_pub = self.create_publisher(String, '/nlp/clarification_needed', 10)
        self._response_pub = self.create_publisher(String, '/nlp/response', 10)

        self.create_subscription(String, '/nlp/text_input', self._on_text, 10)

        self.get_logger().info(
            f'NLP command node ready (model={self._model}, '
            f'{len(self._rooms)} rooms loaded)'
        )

    def _load_rooms(self) -> dict:
        rooms_file = self.get_parameter('rooms_file').value
        if not rooms_file or not os.path.isfile(rooms_file):
            self.get_logger().warn('No rooms file loaded — named navigation disabled')
            return {}
        try:
            import yaml
            with open(rooms_file) as f:
                data = yaml.safe_load(f)
            rooms = data.get('rooms', {})
            self.get_logger().info(f'Loaded {len(rooms)} rooms from {rooms_file}')
            return rooms
        except Exception as e:
            self.get_logger().error(f'Failed to load rooms file: {e}')
            return {}

    def _on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        now = time.monotonic()
        if text == self._last_text and (now - self._last_text_time) < 5.0:
            return
        self._last_text = text
        self._last_text_time = now

        if self._processing:
            self.get_logger().warn(f'Busy processing — dropped: "{text}"')
            return

        self._processing = True
        self.get_logger().info(f'NL input: "{text}"')
        t0 = time.monotonic()

        tool_calls, text_reply, latency = self._call_with_retry(text)

        if tool_calls is None:
            self._processing = False
            return

        tc = tool_calls[0]
        fn_name = tc['function']['name']
        fn_args = tc['function'].get('arguments', {})

        self.get_logger().info(f'LLM tool call: {fn_name}({json.dumps(fn_args)}), latency={latency:.2f}s')

        mission_cmd = self._build_command(fn_name, fn_args, text)
        if mission_cmd:
            self._cmd_pub.publish(mission_cmd)
            self.get_logger().info(f'Published /mission/command: {mission_cmd.command_type}')
            self._log_interaction(text, fn_name, fn_args, latency, 'dispatched')
        else:
            self._log_interaction(text, fn_name, fn_args, latency, 'build_failed')

        if text_reply:
            self._publish_response(text_reply)

        self._processing = False

    def _call_with_retry(self, text: str):
        t0 = time.monotonic()

        for attempt in range(2):
            try:
                if attempt == 0:
                    response = self._call_ollama(text)
                else:
                    response = self._call_ollama(
                        f'You MUST respond with a tool call. The user said: {text}')
            except Exception as e:
                self.get_logger().error(f'Ollama call failed: {e}')
                self._publish_response(f'Error contacting LLM: {e}')
                return None, '', 0.0

            tool_calls = response.get('message', {}).get('tool_calls', [])
            text_reply = response.get('message', {}).get('content', '')

            if tool_calls:
                latency = time.monotonic() - t0
                return tool_calls, text_reply, latency

            if attempt == 0:
                self.get_logger().info('No tool call — retrying with explicit prompt')

        latency = time.monotonic() - t0

        fallback = self._try_keyword_fallback(text)
        if fallback:
            fn_name, fn_args = fallback
            self.get_logger().info(f'Keyword fallback: {fn_name}({json.dumps(fn_args)})')
            mission_cmd = self._build_command(fn_name, fn_args, text)
            if mission_cmd:
                self._cmd_pub.publish(mission_cmd)
                self._log_interaction(text, fn_name, fn_args, latency, 'fallback')
                self._processing = False
            return None, '', latency

        self.get_logger().warn('No tool call after retry — publishing clarification')
        clarify = String()
        clarify.data = text_reply or 'I could not understand that command.'
        self._clarify_pub.publish(clarify)
        self._log_interaction(text, None, None, latency, 'no_tool_call')
        self._publish_response(clarify.data)
        return None, '', latency

    def _try_keyword_fallback(self, text: str):
        lower = text.lower()
        scan_kw = ('scan', 'explore', 'map the', 'look around', 'survey')
        stop_kw = ('stop', 'halt', 'freeze', 'don\'t move', 'emergency')

        for kw in stop_kw:
            if kw in lower:
                return ('stop', {})
        for kw in scan_kw:
            if kw in lower:
                return ('scan_area', {'max_time_s': 120.0, 'mode': 'timed'})
        return None

    def _call_ollama(self, user_text: str) -> dict:
        payload = json.dumps({
            'model': self._model,
            'messages': [
                {'role': 'system', 'content': self._system_prompt},
                {'role': 'user', 'content': user_text},
            ],
            'tools': _TOOL_SCHEMA,
            'stream': False,
        }).encode()

        req = urllib.request.Request(
            f'{self._ollama_url}/api/chat',
            data=payload,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )

        with urllib.request.urlopen(req, timeout=self._timeout) as resp:
            return json.loads(resp.read())

    def _build_command(self, fn_name: str, fn_args: dict, nl_text: str):
        cmd = MissionCommand()
        cmd.command_type = fn_name
        cmd.natural_language = nl_text
        cmd.priority = 1
        cmd.mission_id = f'nlp_{int(time.time())}'

        if fn_name == 'patrol':
            room_ids = fn_args.get('room_ids', fn_args.get('waypoints', []))
            if not self._rooms:
                self._publish_response('No rooms configured. Store rooms first.')
                return None
            resolved = []
            for rid in room_ids:
                if rid in self._rooms:
                    resolved.append({'x': self._rooms[rid]['x'], 'y': self._rooms[rid]['y']})
                else:
                    self._publish_response(
                        f'Unknown room "{rid}". Available: {", ".join(self._rooms.keys())}')
                    return None
            fn_args['waypoints'] = resolved
            fn_args.pop('room_ids', None)

        if fn_name == 'drive_for':
            fn_args = self._fix_distance_params(fn_args, nl_text)

        if fn_name == 'stop':
            cmd.priority = 3

        cmd.parameters_json = json.dumps(fn_args)
        return cmd

    def _fix_distance_params(self, args: dict, nl_text: str) -> dict:
        _DIST_RE = re.compile(
            r'(\d+(?:\.\d+)?)\s*(?:meters?|metres?|m\b)', re.IGNORECASE)
        match = _DIST_RE.search(nl_text)
        if not match:
            return args
        distance = float(match.group(1))
        speed = float(args.get('speed', 0.15))
        if speed <= 0:
            speed = 0.15
        args['duration_s'] = distance / speed
        return args

    def _publish_response(self, text: str):
        msg = String()
        msg.data = text
        self._response_pub.publish(msg)

    def _log_interaction(self, nl_text, tool_name, tool_args, latency, outcome):
        if self._log_file is None:
            return
        entry = {
            'timestamp': time.time(),
            'nl_input': nl_text,
            'tool_name': tool_name,
            'tool_args': tool_args,
            'latency_s': round(latency, 3),
            'outcome': outcome,
            'model': self._model,
        }
        self._log_file.write(json.dumps(entry) + '\n')
        self._log_file.flush()

    def destroy_node(self):
        if self._log_file:
            self._log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NlpCommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
