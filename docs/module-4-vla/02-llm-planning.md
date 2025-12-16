---
sidebar_position: 2
---

# LLM-Based Task Planning

Large Language Models (LLMs) decompose high-level commands into executable robot actions. This chapter integrates GPT-4 or open-source LLMs for intelligent task planning.

## Task Decomposition Pipeline

```
Voice Command → LLM Planner → Action Sequence → Execution Monitor
  "Bring tea"     ["navigate", "grasp", "return"]     ROS 2 Actions
```

## LLM Planning Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')
        openai.api_key = "sk-..."

        self.subscription = self.create_subscription(
            String, '/parsed_command', self.generate_plan, 10
        )

    def generate_plan(self, msg):
        prompt = f"""You are a humanoid robot. Decompose this task into steps:
Task: {msg.data}

Available actions:
- navigate(location): Move to a location
- search(object): Find an object visually
- grasp(object): Pick up an object
- place(object, location): Put object down
- say(text): Speak to user

Respond with JSON: {{"steps": [...]}}"""

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}]
        )

        plan = response.choices[0].message.content
        self.get_logger().info(f"Generated plan: {plan}")
        self.execute_plan(plan)
```

## Executing Plans

```python
import json

def execute_plan(self, plan_json):
    plan = json.loads(plan_json)

    for step in plan["steps"]:
        action = step["action"]
        params = step["params"]

        if action == "navigate":
            self.send_nav_goal(params["location"])
        elif action == "grasp":
            self.send_grasp_goal(params["object"])
        elif action == "say":
            self.speak(params["text"])
```

## Example: "Bring me tea"

**LLM Output:**
```json
{
  "steps": [
    {"action": "navigate", "params": {"location": "kitchen"}},
    {"action": "search", "params": {"object": "tea cup"}},
    {"action": "grasp", "params": {"object": "tea cup"}},
    {"action": "navigate", "params": {"location": "user"}},
    {"action": "place", "params": {"object": "tea cup", "location": "table"}},
    {"action": "say", "params": {"text": "Here is your tea"}}
  ]
}
```

## Monitoring and Recovery

```python
def execute_with_monitoring(self, step):
    success = self.execute_action(step)

    if not success:
        self.get_logger().warn(f"Step failed: {step}")
        recovery = self.llm_recovery(step)
        return self.execute_action(recovery)

def llm_recovery(self, failed_step):
    prompt = f"Action failed: {failed_step}. Suggest recovery action."
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}]
    )
    return response.choices[0].message.content
```

## Conclusion

LLM planning enables robots to understand complex, multi-step tasks from natural language. By combining GPT-4's reasoning with ROS 2's execution infrastructure, humanoid robots achieve unprecedented flexibility. The next chapter integrates everything into a complete capstone system.

## References

- Ahn, M., et al. (2022). "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances." *arXiv preprint arXiv:2204.01691*.
- Brohan, A., et al. (2023). "RT-2: Vision-Language-Action Models." *arXiv preprint arXiv:2307.15818*.
