#!/usr/bin/env python3
"""
Voice Command Node for Humanoid Robot
Integrates Whisper STT with LLM intent parsing and ROS 2 actions

Prerequisites:
- pip install openai-whisper sounddevice numpy openai
- Set OPENAI_API_KEY environment variable

Usage:
    python3 voice_command_node.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import sounddevice as sd
import numpy as np
import whisper
import openai
import os
import tempfile
import wave


class VoiceCommandNode(Node):
    """
    Captures voice commands, transcribes with Whisper,
    parses intent with GPT-4, and executes robot actions
    """

    def __init__(self):
        super().__init__('voice_command_node')

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.command_pub = self.create_publisher(String, '/voice_command', 10)

        # Load Whisper model
        self.get_logger().info('Loading Whisper model...')
        self.whisper_model = whisper.load_model("base")
        self.get_logger().info('Whisper model loaded')

        # OpenAI API setup
        openai.api_key = os.getenv('OPENAI_API_KEY')
        if not openai.api_key:
            self.get_logger().warn('OPENAI_API_KEY not set - LLM parsing disabled')

        # Audio parameters
        self.sample_rate = 16000
        self.duration = 5  # seconds

        self.get_logger().info('Voice command node ready!')
        self.get_logger().info('Say "robot" to activate...')

    def record_audio(self, duration=5):
        """Record audio from microphone"""
        self.get_logger().info(f'Recording for {duration} seconds...')

        recording = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype=np.int16
        )
        sd.wait()

        return recording

    def transcribe_audio(self, audio_data):
        """Transcribe audio using Whisper"""
        # Save to temporary WAV file
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            with wave.open(temp_wav.name, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)  # 16-bit
                wf.setframerate(self.sample_rate)
                wf.writeframes(audio_data.tobytes())

            # Transcribe
            result = self.whisper_model.transcribe(temp_wav.name)
            text = result['text'].strip()

            os.unlink(temp_wav.name)
            return text

    def parse_intent(self, text):
        """Parse natural language intent using GPT-4"""
        if not openai.api_key:
            return None

        system_prompt = """You are a robot command parser. Convert natural language to structured commands.

Available commands:
1. NAVIGATE: {"action": "navigate", "x": float, "y": float, "orientation": float}
2. PICK: {"action": "pick", "object": string}
3. PLACE: {"action": "place", "location": string}
4. STOP: {"action": "stop"}
5. FOLLOW: {"action": "follow", "target": string}

Return ONLY a JSON object, no explanation."""

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"Parse this command: '{text}'"}
                ],
                temperature=0.0
            )

            import json
            intent = json.loads(response.choices[0].message.content)
            return intent

        except Exception as e:
            self.get_logger().error(f'Intent parsing failed: {e}')
            return None

    def execute_command(self, intent):
        """Execute parsed command"""
        if not intent:
            return

        action = intent.get('action')
        self.get_logger().info(f'Executing: {action}')

        if action == 'navigate':
            # Send navigation goal
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = intent.get('x', 0.0)
            goal.pose.position.y = intent.get('y', 0.0)

            # Convert orientation to quaternion
            import math
            yaw = intent.get('orientation', 0.0)
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)

            self.goal_pub.publish(goal)
            self.get_logger().info(f'Navigation goal sent: ({goal.pose.position.x}, {goal.pose.position.y})')

        elif action == 'stop':
            # Publish stop command
            cmd = String()
            cmd.data = 'STOP'
            self.command_pub.publish(cmd)
            self.get_logger().info('Stop command sent')

        elif action in ['pick', 'place', 'follow']:
            # Publish to command topic for other nodes to handle
            cmd = String()
            cmd.data = str(intent)
            self.command_pub.publish(cmd)
            self.get_logger().info(f'{action.capitalize()} command sent')

    def run(self):
        """Main loop - listen for wake word and process commands"""
        while rclpy.ok():
            # Record short clip to detect wake word
            audio = self.record_audio(duration=3)
            text = self.transcribe_audio(audio)

            self.get_logger().info(f'Heard: "{text}"')

            # Check for wake word
            if 'robot' in text.lower():
                self.get_logger().info('Wake word detected! Listening for command...')

                # Record actual command
                audio = self.record_audio(duration=5)
                command_text = self.transcribe_audio(audio)

                self.get_logger().info(f'Command: "{command_text}"')

                # Parse and execute
                intent = self.parse_intent(command_text)
                if intent:
                    self.execute_command(intent)
                else:
                    self.get_logger().warn('Could not parse command')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down voice command node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
