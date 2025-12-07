# LLM Integration for Cognitive Robot Planning

## Learning Objectives

By the end of this lesson, you will be able to:

- Set up OpenAI GPT-4 and Anthropic Claude APIs for robot planning
- Compare LLM capabilities for robotics tasks (GPT-4, Claude, Llama)
- Implement structured output generation (JSON mode, function calling)
- Handle API rate limits, timeouts, and error recovery
- Integrate LLMs with ROS 2 for real-time decision making
- Optimize costs and latency for production deployments

---

## Prerequisites

**Required Knowledge**:
- Python programming (async/await, JSON)
- REST API concepts
- ROS 2 basics (topics, services, actions)
- Completion of [Chapter 11: Voice Recognition](../ch11-whisper/audio-capture.md)

**Required Accounts**:
- OpenAI API key (GPT-4 access) OR
- Anthropic API key (Claude access)
- (Optional) Hugging Face account for open-source models

**Required Software**:
- Python 3.10+
- OpenAI Python SDK (`openai>=1.0.0`)
- Anthropic Python SDK (`anthropic>=0.8.0`)
- ROS 2 Humble

**Estimated Time**: 3-4 hours

---

## Introduction

Large Language Models (LLMs) enable robots to understand complex, free-form natural language commands and generate multi-step action plans. Unlike rule-based parsers (Chapter 11), LLMs can:

- **Handle ambiguity**: "Go get me something to drink" → Plan to find kitchen, locate beverages, select appropriate item
- **Multi-step planning**: "Clean the room" → Pick up objects, place in correct locations, vacuum floor
- **Contextual reasoning**: "Is it safe to move forward?" → Analyze sensor data, evaluate risks, provide explanation
- **Error recovery**: If action fails, propose alternative approaches

**LLM Options for Robotics**:

| Model | Provider | Strengths | Weaknesses | Cost (1M tokens) |
|-------|----------|-----------|------------|------------------|
| **GPT-4 Turbo** | OpenAI | Best reasoning, JSON mode | Expensive, rate limits | $10 (input) / $30 (output) |
| **Claude 3.5 Sonnet** | Anthropic | Long context (200k), fast | No JSON mode | $3 (input) / $15 (output) |
| **GPT-4o mini** | OpenAI | Fast, cheap, good quality | Less capable than GPT-4 | $0.15 (input) / $0.60 (output) |
| **Llama 3 70B** | Meta | Free (self-hosted), privacy | Requires GPU, slower | Free (hardware cost) |

**Recommended for this course**: GPT-4o mini (best cost/performance for learning) or Claude 3.5 Sonnet (best quality/cost for production).

---

## Part 1: OpenAI GPT-4 Setup

### Step 1: Create API Key

1. Sign up at [platform.openai.com](https://platform.openai.com/)
2. Navigate to **API Keys** → **Create new secret key**
3. Name: `robotics-planning`
4. Permissions: `GPT-4` access
5. Copy key: `sk-proj-...`

**Secure storage**:
```bash
echo "OPENAI_API_KEY=sk-proj-..." >> .env
echo ".env" >> .gitignore  # Never commit API keys!
```

---

### Step 2: Install OpenAI SDK

```bash
pip3 install openai python-dotenv

# Verify installation
python3 -c "import openai; print(f'OpenAI SDK v{openai.__version__}')"
```

---

### Step 3: Test API Connection

```python
#!/usr/bin/env python3
import os
from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Test with a simple prompt
response = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system", "content": "You are a helpful robotics assistant."},
        {"role": "user", "content": "Explain what ROS 2 is in one sentence."}
    ],
    max_tokens=100,
    temperature=0.0  # Deterministic output
)

print(response.choices[0].message.content)
```

**Expected output**:
```
ROS 2 is a flexible, modular robotics middleware framework that provides tools and libraries for building robot applications with support for real-time communication, hardware abstraction, and distributed systems.
```

---

## Part 2: Anthropic Claude Setup

### Step 1: Create API Key

1. Sign up at [console.anthropic.com](https://console.anthropic.com/)
2. Navigate to **API Keys** → **Create Key**
3. Name: `robotics-planning`
4. Copy key: `sk-ant-...`

```bash
echo "ANTHROPIC_API_KEY=sk-ant-..." >> .env
```

---

### Step 2: Install Anthropic SDK

```bash
pip3 install anthropic

# Verify
python3 -c "import anthropic; print(f'Anthropic SDK v{anthropic.__version__}')"
```

---

### Step 3: Test API Connection

```python
#!/usr/bin/env python3
import os
from dotenv import load_dotenv
from anthropic import Anthropic

load_dotenv()

client = Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))

response = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    max_tokens=100,
    temperature=0.0,
    messages=[
        {"role": "user", "content": "Explain what ROS 2 is in one sentence."}
    ]
)

print(response.content[0].text)
```

**Expected output**:
```
ROS 2 (Robot Operating System 2) is an open-source middleware framework that provides communication tools, device drivers, and libraries for building and controlling robotic systems across distributed computing environments.
```

---

## Part 3: Structured Output Generation

### Step 1: JSON Mode (GPT-4 only)

Force GPT-4 to output valid JSON:

```python
#!/usr/bin/env python3
from openai import OpenAI
import json
import os

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

response = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {
            "role": "system",
            "content": "You are a robot planning assistant. Output only valid JSON."
        },
        {
            "role": "user",
            "content": "Parse this command into a robot action: 'Move forward 3 meters and pick up the red cube'"
        }
    ],
    response_format={"type": "json_object"},  # Force JSON output
    temperature=0.0
)

# Parse JSON response
result = json.loads(response.choices[0].message.content)
print(json.dumps(result, indent=2))
```

**Output**:
```json
{
  "actions": [
    {
      "type": "move",
      "direction": "forward",
      "distance": 3.0,
      "unit": "meters"
    },
    {
      "type": "pick",
      "object": "cube",
      "color": "red"
    }
  ]
}
```

---

### Step 2: Function Calling (Structured Tool Use)

Define functions LLM can "call" to interact with robot:

```python
#!/usr/bin/env python3
from openai import OpenAI
import json
import os

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Define available robot functions
tools = [
    {
        "type": "function",
        "function": {
            "name": "move_robot",
            "description": "Move the robot in a specified direction",
            "parameters": {
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "enum": ["forward", "backward", "left", "right"],
                        "description": "Direction to move"
                    },
                    "distance": {
                        "type": "number",
                        "description": "Distance in meters"
                    }
                },
                "required": ["direction", "distance"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "pick_object",
            "description": "Pick up an object",
            "parameters": {
                "type": "object",
                "properties": {
                    "object": {
                        "type": "string",
                        "description": "Type of object (e.g., cube, ball, cup)"
                    },
                    "color": {
                        "type": "string",
                        "description": "Color of the object"
                    }
                },
                "required": ["object"]
            }
        }
    }
]

# User command
response = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {
            "role": "user",
            "content": "Move forward 3 meters and pick up the red cube"
        }
    ],
    tools=tools,
    tool_choice="auto"  # Let model decide which functions to call
)

# Extract function calls
for tool_call in response.choices[0].message.tool_calls:
    function_name = tool_call.function.name
    function_args = json.loads(tool_call.function.arguments)

    print(f"Function: {function_name}")
    print(f"Arguments: {json.dumps(function_args, indent=2)}")
    print()
```

**Output**:
```
Function: move_robot
Arguments: {
  "direction": "forward",
  "distance": 3.0
}

Function: pick_object
Arguments: {
  "object": "cube",
  "color": "red"
}
```

---

### Step 3: Prompt Engineering for Structured Output (Claude)

Claude doesn't have native JSON mode, so use prompt engineering:

```python
#!/usr/bin/env python3
from anthropic import Anthropic
import json
import os

client = Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))

system_prompt = """You are a robot planning assistant. You MUST respond with valid JSON only. No explanations, no markdown, just JSON.

Output format:
{
  "actions": [
    {"type": "move", "direction": "forward", "distance": 3.0},
    {"type": "pick", "object": "cube", "color": "red"}
  ]
}"""

response = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    max_tokens=500,
    temperature=0.0,
    system=system_prompt,
    messages=[
        {
            "role": "user",
            "content": "Parse: 'Move forward 3 meters and pick up the red cube'"
        }
    ]
)

# Extract JSON from response
result_text = response.content[0].text
result = json.loads(result_text)
print(json.dumps(result, indent=2))
```

---

## Part 4: ROS 2 Integration

### Step 1: LLM Planning Service

Create a ROS 2 service for LLM-based planning:

**Define service** (`LLMPlan.srv`):
```
# Request
string command  # Natural language command

---

# Response
bool success
string plan_json  # JSON-formatted action plan
string error_message
```

Build:
```bash
colcon build --packages-select my_robot_interfaces
```

---

### Step 2: LLM Planning Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import LLMPlan
from openai import OpenAI
import json
import os
from dotenv import load_dotenv

load_dotenv()

class LLMPlanningNode(Node):
    def __init__(self):
        super().__init__('llm_planning_node')

        # Initialize OpenAI client
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        # Define robot functions
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "navigate",
                    "description": "Navigate to a location or move in a direction",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "action": {
                                "type": "string",
                                "enum": ["move", "turn", "go_to"],
                                "description": "Type of navigation action"
                            },
                            "direction": {
                                "type": "string",
                                "enum": ["forward", "backward", "left", "right"]
                            },
                            "distance": {"type": "number"},
                            "angle": {"type": "number"},
                            "location": {"type": "string"}
                        },
                        "required": ["action"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "manipulate",
                    "description": "Pick, place, or grasp objects",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "action": {
                                "type": "string",
                                "enum": ["pick", "place", "release"]
                            },
                            "object": {"type": "string"},
                            "color": {"type": "string"},
                            "location": {"type": "string"}
                        },
                        "required": ["action", "object"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "perceive",
                    "description": "Look for objects or scan the environment",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "action": {
                                "type": "string",
                                "enum": ["find", "scan", "look_at"]
                            },
                            "object": {"type": "string"}
                        },
                        "required": ["action"]
                    }
                }
            }
        ]

        # Create service
        self.srv = self.create_service(
            LLMPlan,
            'llm_plan',
            self.plan_callback
        )

        self.get_logger().info('LLM Planning Node ready')

    def plan_callback(self, request, response):
        """Generate action plan from natural language command"""
        try:
            command = request.command
            self.get_logger().info(f'Planning for: {command}')

            # Call GPT-4 with function calling
            completion = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {
                        "role": "system",
                        "content": "You are a robot planning assistant. Generate a sequence of actions to complete the user's request."
                    },
                    {
                        "role": "user",
                        "content": command
                    }
                ],
                tools=self.tools,
                tool_choice="auto",
                temperature=0.0
            )

            # Extract tool calls
            tool_calls = completion.choices[0].message.tool_calls

            if not tool_calls:
                response.success = False
                response.error_message = "No valid actions generated"
                return response

            # Convert to JSON plan
            plan = {"actions": []}

            for tool_call in tool_calls:
                action = {
                    "function": tool_call.function.name,
                    "parameters": json.loads(tool_call.function.arguments)
                }
                plan["actions"].append(action)

            response.success = True
            response.plan_json = json.dumps(plan, indent=2)
            response.error_message = ""

            self.get_logger().info(f'Generated {len(plan["actions"])} actions')

        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.get_logger().error(f'Planning failed: {e}')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Step 3: Test LLM Planning Service

Terminal 1 (start node):
```bash
source /opt/ros/humble/setup.bash
export OPENAI_API_KEY="sk-proj-..."
python3 llm_planning_node.py
```

Terminal 2 (call service):
```bash
source /opt/ros/humble/setup.bash

ros2 service call /llm_plan my_robot_interfaces/srv/LLMPlan \
  "{command: 'Go to the kitchen and pick up the red cup'}"
```

**Expected response**:
```yaml
success: true
plan_json: |
  {
    "actions": [
      {
        "function": "navigate",
        "parameters": {
          "action": "go_to",
          "location": "kitchen"
        }
      },
      {
        "function": "perceive",
        "parameters": {
          "action": "find",
          "object": "cup"
        }
      },
      {
        "function": "manipulate",
        "parameters": {
          "action": "pick",
          "object": "cup",
          "color": "red"
        }
      }
    ]
  }
error_message: ''
```

---

## Part 5: Error Handling and Optimization

### Step 1: Retry Logic with Exponential Backoff

```python
import time
from openai import OpenAI, RateLimitError, APIError

def call_llm_with_retry(client, messages, max_retries=3):
    """Call LLM with exponential backoff retry"""
    for attempt in range(max_retries):
        try:
            response = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=messages,
                temperature=0.0,
                max_tokens=500
            )
            return response

        except RateLimitError:
            wait_time = 2 ** attempt  # 1s, 2s, 4s
            print(f"Rate limit hit. Retrying in {wait_time}s...")
            time.sleep(wait_time)

        except APIError as e:
            print(f"API error: {e}")
            if attempt < max_retries - 1:
                time.sleep(1)
            else:
                raise

    raise Exception("Max retries exceeded")
```

---

### Step 2: Timeout Handling

```python
import asyncio
from openai import AsyncOpenAI

async def call_llm_with_timeout(client, messages, timeout=5.0):
    """Call LLM with timeout"""
    try:
        response = await asyncio.wait_for(
            client.chat.completions.create(
                model="gpt-4o-mini",
                messages=messages,
                temperature=0.0
            ),
            timeout=timeout
        )
        return response

    except asyncio.TimeoutError:
        print(f"LLM call timed out after {timeout}s")
        return None

# Usage
async def main():
    client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    messages = [{"role": "user", "content": "Plan: Go to the kitchen"}]

    response = await call_llm_with_timeout(client, messages, timeout=5.0)

    if response:
        print(response.choices[0].message.content)
    else:
        print("Using fallback planner...")

asyncio.run(main())
```

---

### Step 3: Cost Tracking

```python
import tiktoken

class CostTracker:
    def __init__(self, model="gpt-4o-mini"):
        self.model = model
        self.encoding = tiktoken.encoding_for_model(model)

        # Pricing per 1M tokens (as of 2024)
        self.pricing = {
            "gpt-4o-mini": {"input": 0.15, "output": 0.60},
            "gpt-4-turbo": {"input": 10.0, "output": 30.0},
            "claude-3-5-sonnet-20241022": {"input": 3.0, "output": 15.0}
        }

        self.total_input_tokens = 0
        self.total_output_tokens = 0

    def count_tokens(self, text: str) -> int:
        """Count tokens in text"""
        return len(self.encoding.encode(text))

    def log_request(self, messages: list, response):
        """Log token usage from request"""
        # Count input tokens
        input_tokens = sum([
            self.count_tokens(msg['content'])
            for msg in messages
        ])

        # Get output tokens from response
        output_tokens = response.usage.completion_tokens

        self.total_input_tokens += input_tokens
        self.total_output_tokens += output_tokens

        # Calculate cost
        cost = self.calculate_cost()

        print(f"Tokens: {input_tokens} in, {output_tokens} out")
        print(f"Total cost: ${cost:.4f}")

    def calculate_cost(self) -> float:
        """Calculate total cost in USD"""
        input_cost = (self.total_input_tokens / 1_000_000) * self.pricing[self.model]["input"]
        output_cost = (self.total_output_tokens / 1_000_000) * self.pricing[self.model]["output"]
        return input_cost + output_cost

# Usage
tracker = CostTracker(model="gpt-4o-mini")

response = client.chat.completions.create(...)
tracker.log_request(messages, response)
```

---

## Hands-On Exercise

### Exercise 1: Multi-LLM Comparison

Compare GPT-4, Claude, and a local model on the same planning task:

**Requirements**:
- Implement functions for each LLM (GPT-4, Claude, Llama 3)
- Test with 10 robot commands
- Compare: latency, cost, accuracy, JSON validity
- Generate comparison table

**Starter code**:

```python
import time

class LLMBenchmark:
    def __init__(self):
        self.gpt_client = OpenAI(...)
        self.claude_client = Anthropic(...)

    def benchmark(self, command: str) -> dict:
        """Benchmark all LLMs on a command"""
        results = {}

        # GPT-4
        start = time.time()
        gpt_response = self.gpt_client.chat.completions.create(...)
        results['gpt4'] = {
            'latency': time.time() - start,
            'output': gpt_response.choices[0].message.content
        }

        # TODO: Add Claude
        # TODO: Add local model (Llama 3)
        # TODO: Calculate costs
        # TODO: Validate JSON

        return results

# Test commands
commands = [
    "Go to the kitchen",
    "Pick up the red cube and place it on the table",
    # ... 8 more
]
```

---

### Exercise 2: Caching for Repeated Commands

Implement a cache to avoid re-calling LLM for identical commands:

**Requirements**:
- Hash input command
- Check cache before calling LLM
- Store response with TTL (time-to-live)
- Track cache hit rate

**Starter code**:

```python
import hashlib
import time

class LLMCache:
    def __init__(self, ttl=3600):
        self.cache = {}  # {hash: (response, timestamp)}
        self.ttl = ttl

    def get(self, command: str):
        # TODO: Hash command
        # TODO: Check if in cache and not expired
        # TODO: Return cached response or None
        pass

    def set(self, command: str, response: str):
        # TODO: Hash command
        # TODO: Store with timestamp
        pass

# Usage
cache = LLMCache(ttl=3600)  # 1 hour TTL

cached = cache.get(command)
if cached:
    print("Cache hit!")
    response = cached
else:
    response = client.chat.completions.create(...)
    cache.set(command, response)
```

---

## Summary

In this lesson, you learned to:

- ✅ Set up OpenAI GPT-4 and Anthropic Claude APIs
- ✅ Generate structured JSON output with JSON mode and function calling
- ✅ Integrate LLMs with ROS 2 services for cognitive planning
- ✅ Handle errors, timeouts, and rate limits
- ✅ Track costs and optimize token usage
- ✅ Compare LLM options for robotics applications

**Key Takeaways**:
- **GPT-4o mini**: Best for learning (cheap, fast, good quality)
- **Claude 3.5 Sonnet**: Best for production (long context, lower cost)
- **Function calling**: Most reliable way to get structured actions
- **Cost optimization**: Cache frequent commands, use shorter prompts
- **Error handling**: Always implement retries and timeouts

---

## Additional Resources

### Official Documentation
- [OpenAI API Reference](https://platform.openai.com/docs/api-reference)
- [Anthropic Claude API](https://docs.anthropic.com/claude/reference/getting-started)
- [Function Calling Guide](https://platform.openai.com/docs/guides/function-calling)

### Cost Optimization
- [tiktoken (Token Counter)](https://github.com/openai/tiktoken)
- [OpenAI Pricing Calculator](https://openai.com/pricing)
- [Anthropic Pricing](https://www.anthropic.com/pricing)

### Research Papers
- [Language Models as Zero-Shot Planners](https://arxiv.org/abs/2201.07207)
- [Do As I Can, Not As I Say (SayCan)](https://arxiv.org/abs/2204.01691) - Google's LLM for robotics

---

## Next Lesson

In **Lesson 2: Prompt Engineering**, you'll learn to:
- Design robotics-specific prompts for consistent output
- Use few-shot examples to improve planning quality
- Handle edge cases and error scenarios
- Build a prompt library for common tasks

Continue to [Prompt Engineering →](./prompt-engineering.md)
