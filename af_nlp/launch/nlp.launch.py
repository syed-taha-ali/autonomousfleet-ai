#!/usr/bin/env python3
"""NLP mission interface — runs on the Dev PC.

Brings up nlp_command_node which listens on /nlp/text_input, calls
local Ollama, and publishes MissionCommand on /mission/command.

Usage:
  ros2 launch af_nlp nlp.launch.py

  # Then send text commands:
  ros2 topic pub --once /nlp/text_input std_msgs/String '{data: "find the suitcase"}'
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='qwen2.5:7b-instruct-q4_K_M',
            description='Ollama model tag to use for NL translation.',
        ),
        DeclareLaunchArgument(
            'ollama_url', default_value='http://localhost:11434',
            description='Ollama server URL.',
        ),
        DeclareLaunchArgument(
            'log_file', default_value='',
            description='Path to JSONL log file for NL interactions (empty = no logging).',
        ),

        Node(
            package='af_nlp',
            executable='nlp_command_node',
            name='nlp_command',
            output='screen',
            parameters=[{
                'model': LaunchConfiguration('model'),
                'ollama_url': LaunchConfiguration('ollama_url'),
                'rooms_file': PathJoinSubstitution([
                    FindPackageShare('af_nlp'), 'config', 'rooms.yaml']),
                'log_file': LaunchConfiguration('log_file'),
                'timeout_s': 30.0,
            }],
        ),
    ])
