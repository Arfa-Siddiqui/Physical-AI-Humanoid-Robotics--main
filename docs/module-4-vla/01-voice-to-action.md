---
sidebar_position: 1
---

# Voice-to-Action: Integrating Speech Recognition

Voice control transforms robots from programmed automatons to natural language-driven assistants. This chapter integrates Whisper for real-time speech recognition with ROS 2 action servers.

## Architecture Overview

```
Microphone → Whisper (STT) → Intent Parser → ROS 2 Action Client → Robot
```

**Components:**
- **Audio Capture**: ROS 2 node subscribing to microphone
- **Whisper STT**: Transcribes speech to text
- **Intent Parser**: Extracts commands (navigate, grasp, search)
- **Action Client**: Sends goals to navigation/manipulation servers

## Audio Capture Node

Create `audio_capture_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import wave

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture')
        self.publisher_ = self.create_publisher(String, '/voice_command', 10)

        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.audio = pyaudio.PyAudio()

        self.get_logger().info("Say 'robot' to activate...")

    def listen_for_wake_word(self):
        stream = self.audio.open(format=self.FORMAT, channels=self.CHANNELS,
                                rate=self.RATE, input=True, frames_per_buffer=self.CHUNK)
        frames = []

        for _ in range(0, int(self.RATE / self.CHUNK * 3)):  # 3 seconds
            data = stream.read(self.CHUNK)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        # Save to temp file for Whisper
        wf = wave.open("/tmp/command.wav", 'wb')
        wf.setnchannels(self.CHANNELS)
        wf.setsampwidth(self.audio.get_sample_size(self.FORMAT))
        wf.setframerate(self.RATE)
        wf.writeframes(b''.join(frames))
        wf.close()

        return "/tmp/command.wav"
```

## Whisper Integration

```python
import whisper

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command')
        self.model = whisper.load_model("base")
        self.publisher_ = self.create_publisher(String, '/parsed_command', 10)

        self.subscription = self.create_subscription(
            String,
            '/voice_command',
            self.process_audio,
            10
        )

    def process_audio(self, msg):
        result = self.model.transcribe(msg.data)
        text = result["text"]
        self.get_logger().info(f"Heard: {text}")

        # Publish transcribed text
        command = String()
        command.data = text
        self.publisher_.publish(command)
```

## Intent Parsing

Extract actionable commands:

```python
import re

class IntentParser(Node):
    def __init__(self):
        super().__init__('intent_parser')
        self.subscription = self.create_subscription(
            String,
            '/parsed_command',
            self.parse_intent,
            10
        )

    def parse_intent(self, msg):
        text = msg.data.lower()

        # Navigation intent
        if re.search(r'go to|navigate to|move to', text):
            location = self.extract_location(text)
            self.send_navigation_goal(location)

        # Grasping intent
        elif re.search(r'pick up|grab|grasp', text):
            object_name = self.extract_object(text)
            self.send_grasp_goal(object_name)

        # Search intent
        elif re.search(r'find|locate|search for', text):
            object_name = self.extract_object(text)
            self.send_search_goal(object_name)

    def extract_location(self, text):
        # Simple keyword extraction (can use NER models for robustness)
        locations = ["kitchen", "bedroom", "living room", "office"]
        for loc in locations:
            if loc in text:
                return loc
        return "unknown"
```

## Action Integration

Send parsed commands to ROS 2 action servers:

```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

def send_navigation_goal(self, location):
    # Map locations to coordinates
    coords = {
        "kitchen": (5.0, 3.0),
        "bedroom": (-2.0, -1.0),
        "living room": (0.0, 0.0)
    }

    if location not in coords:
        self.get_logger().warn(f"Unknown location: {location}")
        return

    x, y = coords[location]

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation.w = 1.0

    self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    self._nav_client.wait_for_server()
    self._nav_client.send_goal_async(goal_msg)

    self.get_logger().info(f"Navigating to {location}")
```

## Complete Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vla_system',
            executable='audio_capture_node',
            name='audio_capture'
        ),
        Node(
            package='vla_system',
            executable='voice_command_node',
            name='voice_command'
        ),
        Node(
            package='vla_system',
            executable='intent_parser',
            name='intent_parser'
        )
    ])
```

## Testing

```bash
# Terminal 1: Launch VLA nodes
ros2 launch vla_system voice_control.launch.py

# Terminal 2: Monitor commands
ros2 topic echo /parsed_command

# Speak into microphone: "Go to the kitchen"
# Expected: Navigation goal sent to Nav2
```

## Advanced: Continuous Listening

```python
def continuous_listen(self):
    while rclpy.ok():
        audio_file = self.listen_for_wake_word()
        result = self.model.transcribe(audio_file)

        if "robot" in result["text"].lower():
            self.get_logger().info("Wake word detected. Listening for command...")
            command_file = self.listen_for_command()
            command = self.model.transcribe(command_file)
            self.publish_command(command["text"])
```

## Conclusion

Voice control bridges human intent and robot action. By combining Whisper for accurate transcription with intent parsing and ROS 2 actions, humanoid robots respond naturally to spoken commands. In the next chapter, we'll add LLM-based planning for complex, multi-step tasks.

## References

- Radford, A., et al. (2022). "Robust Speech Recognition via Large-Scale Weak Supervision." OpenAI.
- Macenski, S. (2024). "Nav2 Action Server Documentation." Retrieved from https://navigation.ros.org/
