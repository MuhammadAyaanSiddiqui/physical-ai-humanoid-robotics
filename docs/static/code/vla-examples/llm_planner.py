#!/usr/bin/env python3
"""
LLM Planner Example

Uses GPT-4 to generate robot action plans from natural language commands.
"""

import os
import json
from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

SYSTEM_PROMPT = """You are a robot planning assistant. Generate action plans in JSON format.

Available actions:
- navigate: {action: "navigate", location: "kitchen"}
- pick: {action: "pick", object: "cup", color: "red"}
- place: {action: "place", object: "cup", location: "table"}
- scan: {action: "scan"}

Output format:
{
  "plan": [
    {"action": "navigate", "location": "kitchen"},
    {"action": "pick", "object": "cup"}
  ]
}
"""

def generate_plan(command):
    """Generate action plan from natural language command"""
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": f"Command: {command}"}
        ],
        response_format={"type": "json_object"},
        temperature=0.0
    )

    plan = json.loads(response.choices[0].message.content)
    return plan

def main():
    """Test LLM planner"""
    commands = [
        "Go to the kitchen",
        "Pick up the red cup",
        "Go to the kitchen and pick up the red cup",
        "Find the blue ball and bring it to me"
    ]

    for cmd in commands:
        print(f"\nCommand: {cmd}")
        plan = generate_plan(cmd)
        print(f"Plan: {json.dumps(plan, indent=2)}")

if __name__ == "__main__":
    main()
