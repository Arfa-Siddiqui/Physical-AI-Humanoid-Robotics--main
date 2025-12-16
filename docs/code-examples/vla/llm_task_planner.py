#!/usr/bin/env python3
"""
LLM Task Planner for Humanoid Robot
Uses GPT-4 to decompose high-level tasks into robot actions

Prerequisites:
- pip install openai
- Set OPENAI_API_KEY environment variable

Usage:
    python3 llm_task_planner.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os
import json


class LLMTaskPlanner(Node):
    """
    Decomposes high-level natural language tasks into
    executable robot primitive actions
    """

    def __init__(self):
        super().__init__('llm_task_planner')

        # Setup OpenAI
        openai.api_key = os.getenv('OPENAI_API_KEY')
        if not openai.api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError('OpenAI API key required')

        # Subscriber for high-level tasks
        self.task_sub = self.create_subscription(
            String,
            '/high_level_task',
            self.task_callback,
            10
        )

        # Publisher for action sequence
        self.action_pub = self.create_publisher(String, '/action_sequence', 10)

        # Robot capabilities
        self.capabilities = {
            "navigate": "Move to a location (x, y, orientation)",
            "pick": "Pick up an object",
            "place": "Place held object at location",
            "open_door": "Open a door",
            "press_button": "Press a button",
            "wait": "Wait for a duration",
            "observe": "Look around and identify objects"
        }

        self.get_logger().info('LLM Task Planner ready')
        self.get_logger().info(f'Capabilities: {list(self.capabilities.keys())}')

    def task_callback(self, msg):
        """Process incoming high-level task"""
        task = msg.data
        self.get_logger().info(f'Received task: "{task}"')

        # Plan action sequence
        action_sequence = self.plan_task(task)

        if action_sequence:
            self.get_logger().info(f'Generated {len(action_sequence)} actions')
            self.publish_actions(action_sequence)
        else:
            self.get_logger().error('Failed to generate action sequence')

    def plan_task(self, task_description):
        """Use GPT-4 to decompose task into action sequence"""

        system_prompt = f"""You are a robot task planner. Given a high-level task, decompose it into a sequence of primitive actions.

Available actions:
{json.dumps(self.capabilities, indent=2)}

Return a JSON array of actions. Each action is:
{{
  "action": "action_name",
  "parameters": {{...}},
  "description": "what this step does"
}}

Example:
Task: "Bring me a water bottle from the kitchen"
Output:
[
  {{"action": "navigate", "parameters": {{"x": 5.0, "y": 2.0}}, "description": "Go to kitchen"}},
  {{"action": "observe", "parameters": {{}}, "description": "Look for water bottle"}},
  {{"action": "navigate", "parameters": {{"x": 5.2, "y": 2.1}}, "description": "Move to bottle"}},
  {{"action": "pick", "parameters": {{"object": "water_bottle"}}, "description": "Pick up bottle"}},
  {{"action": "navigate", "parameters": {{"x": 0.0, "y": 0.0}}, "description": "Return to user"}},
  {{"action": "place", "parameters": {{"location": "table"}}, "description": "Place bottle on table"}}
]

Return ONLY the JSON array, no explanation."""

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"Plan this task: {task_description}"}
                ],
                temperature=0.0
            )

            plan_text = response.choices[0].message.content.strip()
            plan = json.loads(plan_text)

            # Validate plan
            for step in plan:
                if 'action' not in step or 'parameters' not in step:
                    self.get_logger().error(f'Invalid action: {step}')
                    return None

            return plan

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse LLM response: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Planning failed: {e}')
            return None

    def publish_actions(self, action_sequence):
        """Publish action sequence for execution"""
        msg = String()
        msg.data = json.dumps(action_sequence)
        self.action_pub.publish(msg)

        # Log plan
        self.get_logger().info('Action Plan:')
        for i, action in enumerate(action_sequence, 1):
            self.get_logger().info(
                f"  {i}. {action['action']}: {action.get('description', '')}"
            )


def main(args=None):
    rclpy.init(args=args)
    planner = LLMTaskPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Shutting down task planner')
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
