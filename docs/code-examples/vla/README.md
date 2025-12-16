# Vision-Language-Action (VLA) Integration Scripts

Complete VLA pipeline for natural language robot control using Whisper, GPT-4, and ROS 2.

## Overview

The VLA system enables natural language control of humanoid robots:

1. **Voice Input** → Whisper STT transcribes speech
2. **Intent Parsing** → GPT-4 extracts structured commands
3. **Task Planning** → LLM decomposes complex tasks into primitives
4. **Action Execution** → ROS 2 nodes execute robot actions

## Prerequisites

### Software Requirements
- ROS 2 Humble
- Python 3.8+
- OpenAI API key (for GPT-4)

### Python Dependencies
```bash
pip install openai-whisper sounddevice numpy openai
```

### System Dependencies
```bash
# For audio recording
sudo apt install portaudio19-dev python3-pyaudio

# For ROS 2
sudo apt install ros-humble-navigation2
```

### API Keys
```bash
# Set OpenAI API key
export OPENAI_API_KEY="sk-your-api-key-here"

# Or add to ~/.bashrc:
echo 'export OPENAI_API_KEY="sk-your-key"' >> ~/.bashrc
source ~/.bashrc
```

Get API key from: https://platform.openai.com/api-keys

## Files

### 1. `voice_command_node.py`
Real-time voice control system with Whisper and GPT-4

**Features:**
- Wake word detection ("robot")
- Speech-to-text with Whisper
- Intent parsing with GPT-4
- Publishes navigation goals and commands to ROS 2

**Supported commands:**
- "Robot, go to the kitchen" → Navigation
- "Robot, pick up the cup" → Manipulation
- "Robot, stop" → Emergency stop
- "Robot, follow me" → Following behavior

**Run:**
```bash
python3 voice_command_node.py
```

**Test without robot:**
```bash
# Terminal 1: Run voice node
python3 voice_command_node.py

# Terminal 2: Monitor commands
ros2 topic echo /voice_command
ros2 topic echo /goal_pose
```

### 2. `llm_task_planner.py`
High-level task decomposition using GPT-4

**Features:**
- Receives natural language task descriptions
- Decomposes into action sequences
- Uses robot's known capabilities
- Publishes action plan for execution

**Example task:**
```
Input: "Bring me a water bottle from the kitchen"

Output action sequence:
1. navigate: Go to kitchen (5.0, 2.0)
2. observe: Look for water bottle
3. navigate: Move to bottle (5.2, 2.1)
4. pick: Pick up bottle
5. navigate: Return to user (0.0, 0.0)
6. place: Place on table
```

**Run:**
```bash
python3 llm_task_planner.py
```

**Send task:**
```bash
ros2 topic pub --once /high_level_task std_msgs/String \
  "data: 'Bring me a water bottle from the kitchen'"
```

## Complete VLA System Setup

### Architecture

```
Microphone → Voice Command Node → Task Planner → Action Executor → Robot
     ↓                ↓                  ↓              ↓
  Whisper STT     GPT-4 Intent      GPT-4 Plan    ROS 2 Actions
```

### Launch Full System

```bash
# Terminal 1: Robot simulation (Gazebo/Isaac)
ros2 launch your_package simulation.launch.py

# Terminal 2: Navigation stack
ros2 launch nav2_bringup navigation_launch.py params_file:=nav2_params.yaml

# Terminal 3: Task planner
python3 llm_task_planner.py

# Terminal 4: Voice command node
python3 voice_command_node.py

# Terminal 5: RViz visualization
ros2 launch nav2_bringup rviz_launch.py
```

## Usage Examples

### Example 1: Simple Navigation
```
User: "Robot, go to the kitchen"
→ Whisper transcribes → GPT-4 parses → Publishes /goal_pose
```

### Example 2: Complex Task
```
User publishes to /high_level_task: "Clean the living room"

Task Planner decomposes:
1. navigate → living room
2. observe → identify objects
3. pick → item 1
4. navigate → trash
5. place → trash
6. [repeat for other items]
```

### Example 3: Interactive Commands
```python
# Custom interaction script
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class InteractiveCommands(Node):
    def __init__(self):
        super().__init__('interactive_commands')
        self.pub = self.create_publisher(String, '/high_level_task', 10)

    def send_task(self, task):
        msg = String()
        msg.data = task
        self.pub.publish(msg)
        self.get_logger().info(f'Sent task: {task}')

def main():
    rclpy.init()
    node = InteractiveCommands()

    tasks = [
        "Survey the room and report objects",
        "Navigate to the red box",
        "Pick up the red box",
        "Bring it to me"
    ]

    for task in tasks:
        node.send_task(task)
        input("Press Enter for next task...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Customization

### Add New Actions
Edit `llm_task_planner.py`:
```python
self.capabilities = {
    "navigate": "Move to a location",
    "pick": "Pick up an object",
    "place": "Place object",
    # Add custom actions:
    "open_drawer": "Open a drawer by handle",
    "scan_barcode": "Read barcode with camera",
    "charge": "Go to charging station"
}
```

### Change LLM Model
```python
# Use GPT-3.5-turbo (faster, cheaper)
response = openai.ChatCompletion.create(
    model="gpt-3.5-turbo",  # Changed from gpt-4
    ...
)
```

### Adjust Whisper Model
```python
# Options: tiny, base, small, medium, large
# Larger = more accurate but slower
self.whisper_model = whisper.load_model("small")
```

## Troubleshooting

### "OPENAI_API_KEY not set"
```bash
export OPENAI_API_KEY="your-key-here"
# Verify:
echo $OPENAI_API_KEY
```

### Microphone not working
```bash
# List audio devices
python3 -c "import sounddevice; print(sounddevice.query_devices())"

# Test recording
python3 -c "import sounddevice as sd; import numpy as np; \
  recording = sd.rec(int(3*16000), samplerate=16000, channels=1); \
  sd.wait(); print('Recorded successfully')"
```

### Whisper model download fails
```bash
# Manually download models
mkdir -p ~/.cache/whisper
cd ~/.cache/whisper
wget https://openaipublic.azureedge.net/main/whisper/models/.../base.pt
```

### GPT-4 API errors
- **Rate limit:** Add delay between requests
- **Quota exceeded:** Check OpenAI dashboard
- **Invalid key:** Regenerate API key

### Voice commands not recognized
- Speak clearly and slowly
- Increase `duration` in `record_audio()`
- Use larger Whisper model (`medium` or `large`)
- Check microphone volume

## Cost Estimation

### OpenAI API Costs (as of 2024)
- **GPT-4:** ~$0.03 per 1K tokens
- **GPT-3.5-turbo:** ~$0.002 per 1K tokens

### Typical Usage
- Voice command parsing: ~100-200 tokens per command
- Task planning: ~500-1000 tokens per task
- **Estimated cost:** $0.01-0.05 per voice interaction with GPT-4

### Cost Optimization
1. Use GPT-3.5-turbo for simple commands
2. Cache common command patterns
3. Use local models for intent classification
4. Implement command templates for frequent tasks

## Integration with Book Examples

This VLA system integrates with:
- **ROS 2 examples** (`../ros2/`) - Uses same topics and nodes
- **Nav2 configs** (`../nav2/`) - Navigation goals sent to Nav2
- **Isaac Sim** (`../isaac/`) - Works with Isaac ROS bridge
- **Gazebo** (`../gazebo/`) - Compatible with Gazebo simulations

## Related Book Chapters

- **Module 4.1**: Voice-to-Action Pipeline
- **Module 4.2**: LLM-Based Task Planning
- **Module 4.3**: Capstone Integration Project
- **Module 3.3**: Nav2 (navigation backend)

## Resources

- [OpenAI Whisper](https://github.com/openai/whisper)
- [OpenAI API Documentation](https://platform.openai.com/docs)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Vision-Language Models](https://arxiv.org/abs/2204.02311)
- [Embodied AI](https://embodied-ai.org/)
