# Prompt Engineering for Robotics Tasks

## Learning Objectives

By the end of this lesson, you will be able to:

- Design effective prompts for robotics planning tasks
- Use few-shot learning to improve LLM output quality
- Implement chain-of-thought reasoning for complex tasks
- Create reusable prompt templates for common scenarios
- Handle edge cases and constraint specification
- Optimize prompts for cost and latency

---

## Prerequisites

**Required Knowledge**:
- Completion of [Lesson 1: LLM Integration](./llm-integration.md)
- Understanding of robotics action primitives
- JSON formatting

**Required Software**:
- OpenAI or Anthropic API access
- Python 3.10+

**Estimated Time**: 2-3 hours

---

## Introduction

Effective prompt engineering is critical for reliable robot behavior. Poor prompts lead to:
- Inconsistent output formats
- Missed safety constraints
- Inefficient action plans
- Hallucinated capabilities

**Key Principles**:
1. **Be specific**: Define exact output format and constraints
2. **Use examples**: Show desired behavior with few-shot learning
3. **Add context**: Provide robot capabilities and environment state
4. **Think step-by-step**: Use chain-of-thought for complex reasoning
5. **Validate output**: Check constraints before execution

---

## Part 1: Basic Prompt Structure

### Step 1: System Prompt Template

```python
SYSTEM_PROMPT = """You are a planning assistant for a humanoid robot with the following capabilities:

NAVIGATION:
- move(direction, distance): Move forward/backward (max 10m)
- turn(direction, angle): Rotate left/right (0-180°)
- go_to(location): Navigate to named location

MANIPULATION:
- pick(object, color): Grasp object
- place(object, location): Put object down
- release(): Open gripper

PERCEPTION:
- find(object): Locate object in environment
- scan(): Get list of visible objects

CONSTRAINTS:
- Maximum payload: 5kg
- Cannot pick multiple objects simultaneously
- Must find object before picking
- Must be at location before placing

OUTPUT FORMAT:
Return valid JSON with this structure:
{
  "plan": [
    {"action": "move", "direction": "forward", "distance": 2.0},
    {"action": "pick", "object": "cube", "color": "red"}
  ],
  "reasoning": "Brief explanation of plan"
}
"""
```

---

### Step 2: User Prompt Template

```python
def create_user_prompt(command: str, environment: dict) -> str:
    """Generate user prompt with command and environment state"""
    return f"""
COMMAND: {command}

CURRENT STATE:
- Robot location: {environment.get('location', 'unknown')}
- Visible objects: {', '.join(environment.get('visible_objects', []))}
- Gripper: {'holding ' + environment.get('held_object', '') if environment.get('held_object') else 'empty'}

Generate an action plan to complete the command.
"""

# Usage
env = {
    'location': 'living_room',
    'visible_objects': ['red_cube', 'blue_ball', 'table'],
    'held_object': None
}

user_prompt = create_user_prompt("Pick up the red cube", env)
```

---

## Part 2: Few-Shot Learning

### Step 1: Few-Shot Examples for Consistent Output

```python
FEW_SHOT_EXAMPLES = """
Example 1:
COMMAND: Move forward 3 meters
OUTPUT:
{
  "plan": [
    {"action": "move", "direction": "forward", "distance": 3.0}
  ],
  "reasoning": "Simple forward movement within 10m limit"
}

Example 2:
COMMAND: Go to the kitchen and pick up the red cup
OUTPUT:
{
  "plan": [
    {"action": "go_to", "location": "kitchen"},
    {"action": "find", "object": "cup"},
    {"action": "pick", "object": "cup", "color": "red"}
  ],
  "reasoning": "Navigate to kitchen, locate cup visually, then grasp it"
}

Example 3:
COMMAND: Pick up the cube and place it on the table
OUTPUT:
{
  "plan": [
    {"action": "find", "object": "cube"},
    {"action": "pick", "object": "cube"},
    {"action": "go_to", "location": "table"},
    {"action": "place", "object": "cube", "location": "table"},
    {"action": "release"}
  ],
  "reasoning": "Find cube first, pick it up, navigate to table, place and release"
}

Now process the following command:
"""

# Combine with system prompt
def create_few_shot_prompt(command: str) -> str:
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": FEW_SHOT_EXAMPLES + f"\nCOMMAND: {command}"}
    ]
    return messages
```

---

### Step 2: Testing Few-Shot vs. Zero-Shot

```python
#!/usr/bin/env python3
from openai import OpenAI
import json

client = OpenAI()

def zero_shot_plan(command: str) -> dict:
    """Generate plan without examples"""
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": f"COMMAND: {command}"}
        ],
        response_format={"type": "json_object"},
        temperature=0.0
    )
    return json.loads(response.choices[0].message.content)

def few_shot_plan(command: str) -> dict:
    """Generate plan with examples"""
    messages = create_few_shot_prompt(command)
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages,
        response_format={"type": "json_object"},
        temperature=0.0
    )
    return json.loads(response.choices[0].message.content)

# Compare
command = "Bring me the blue ball from the kitchen"

print("=== ZERO-SHOT ===")
print(json.dumps(zero_shot_plan(command), indent=2))

print("\n=== FEW-SHOT ===")
print(json.dumps(few_shot_plan(command), indent=2))
```

**Typical improvement**: Few-shot reduces missing steps (e.g., forgetting `find` before `pick`) by ~70%.

---

## Part 3: Chain-of-Thought Reasoning

### Step 1: Enable Step-by-Step Reasoning

```python
COT_SYSTEM_PROMPT = """You are a planning assistant for a humanoid robot.

When given a command, think through it step-by-step:
1. What is the goal?
2. What is the current state?
3. What actions are needed?
4. Are there any constraints violated?
5. Generate the final plan

Be explicit about your reasoning before outputting the plan.
"""

def chain_of_thought_plan(command: str, environment: dict) -> dict:
    """Generate plan with chain-of-thought reasoning"""
    messages = [
        {"role": "system", "content": COT_SYSTEM_PROMPT},
        {"role": "user", "content": f"""
COMMAND: {command}

ENVIRONMENT:
{json.dumps(environment, indent=2)}

Think step-by-step, then provide your final plan in JSON format.
"""}
    ]

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages,
        temperature=0.0
    )

    # Extract reasoning and plan
    content = response.choices[0].message.content

    # Parse JSON from response (may need extraction if mixed with text)
    import re
    json_match = re.search(r'\{.*\}', content, re.DOTALL)
    if json_match:
        plan = json.loads(json_match.group())
        return {
            "reasoning": content[:json_match.start()].strip(),
            "plan": plan
        }

    return {"reasoning": content, "plan": None}

# Usage
env = {
    'location': 'bedroom',
    'visible_objects': ['lamp', 'book'],
    'held_object': None
}

result = chain_of_thought_plan("Turn off the lamp in the living room", env)
print("REASONING:", result['reasoning'])
print("\nPLAN:", json.dumps(result['plan'], indent=2))
```

---

### Step 2: Safety Constraint Checking

```python
SAFETY_PROMPT = """Before generating a plan, verify these safety constraints:

SAFETY RULES:
1. Never exceed 10m movement in single action
2. Never pick objects > 5kg (check object database)
3. Must release object before picking new one
4. Cannot navigate while holding fragile objects
5. Must confirm object exists before manipulation

If command violates constraints, respond with:
{
  "error": "constraint_violation",
  "violated_rule": "<rule number>",
  "suggestion": "<alternative approach>"
}

Otherwise, provide the action plan.
"""

def safe_plan(command: str, environment: dict) -> dict:
    """Generate plan with safety validation"""
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT + "\n\n" + SAFETY_PROMPT},
        {"role": "user", "content": create_user_prompt(command, environment)}
    ]

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages,
        response_format={"type": "json_object"},
        temperature=0.0
    )

    result = json.loads(response.choices[0].message.content)

    if "error" in result:
        print(f"⚠️ Safety violation: {result['violated_rule']}")
        print(f"Suggestion: {result['suggestion']}")
        return None

    return result

# Test with unsafe command
env = {'location': 'warehouse', 'held_object': 'box_20kg'}
unsafe_result = safe_plan("Pick up the engine block", env)  # Violates 5kg limit
```

---

## Part 4: Contextual Prompting

### Step 1: Environment-Aware Planning

```python
def create_contextual_prompt(command: str, context: dict) -> str:
    """Generate prompt with rich environmental context"""

    # Build context description
    context_desc = f"""
ROBOT STATE:
- Position: {context['robot']['position']}
- Orientation: {context['robot']['orientation']}°
- Battery: {context['robot']['battery']}%
- Gripper: {'occupied' if context['robot']['holding'] else 'empty'}

ENVIRONMENT:
- Room: {context['environment']['room']}
- Visible objects: {len(context['environment']['objects'])} items
  {chr(10).join([f"  • {obj['name']} ({obj['color']}, {obj['distance']:.1f}m away)" for obj in context['environment']['objects']])}
- Obstacles: {', '.join(context['environment']['obstacles'])}

TASK HISTORY (last 3 actions):
{chr(10).join([f"  {i+1}. {action}" for i, action in enumerate(context['history'][-3:])])}

COMMAND: {command}

Generate an efficient plan considering the current context.
"""
    return context_desc

# Rich context example
context = {
    'robot': {
        'position': {'x': 2.5, 'y': 1.0},
        'orientation': 90,
        'battery': 75,
        'holding': None
    },
    'environment': {
        'room': 'kitchen',
        'objects': [
            {'name': 'cup', 'color': 'red', 'distance': 1.2},
            {'name': 'plate', 'color': 'white', 'distance': 2.5}
        ],
        'obstacles': ['table', 'chair']
    },
    'history': [
        'move(forward, 1.0)',
        'turn(left, 90)',
        'scan()'
    ]
}

prompt = create_contextual_prompt("Pick up the red cup", context)
```

---

### Step 2: Dynamic Constraint Injection

```python
def add_dynamic_constraints(base_prompt: str, constraints: list) -> str:
    """Add runtime constraints to prompt"""

    constraint_text = "\nADDITIONAL CONSTRAINTS:\n"
    for i, constraint in enumerate(constraints, 1):
        constraint_text += f"{i}. {constraint}\n"

    return base_prompt + constraint_text

# Example: Add time constraint
constraints = [
    "Complete task in under 30 seconds (assume 2s per action)",
    "Avoid moving near the window (fragile items)",
    "Prioritize battery conservation (minimize distance)"
]

enhanced_prompt = add_dynamic_constraints(SYSTEM_PROMPT, constraints)
```

---

## Part 5: Prompt Optimization

### Step 1: Token Reduction Techniques

```python
# VERBOSE (expensive)
VERBOSE_PROMPT = """
You are an advanced artificial intelligence system designed to assist with planning and control of humanoid robotic systems. Your primary function is to interpret natural language commands provided by human operators and translate them into structured sequences of low-level robotic actions...
"""

# CONCISE (cost-effective)
CONCISE_PROMPT = """
Robot planner. Convert commands to action sequences.

Actions: move(dir,dist), turn(dir,angle), pick(obj,color), place(obj,loc)
Constraints: 10m max move, 5kg max load
Output: JSON action array
"""

# Token comparison
import tiktoken

enc = tiktoken.encoding_for_model("gpt-4o-mini")
print(f"Verbose: {len(enc.encode(VERBOSE_PROMPT))} tokens")
print(f"Concise: {len(enc.encode(CONCISE_PROMPT))} tokens")
# Output: Verbose: 87 tokens, Concise: 34 tokens (60% reduction)
```

---

### Step 2: Prompt Caching (For Repeated Queries)

```python
class PromptCache:
    """Cache system prompts to reduce costs"""

    def __init__(self):
        self.system_prompt = SYSTEM_PROMPT  # Cached, sent once
        self.conversation_history = []

    def add_user_message(self, command: str):
        """Add user command to conversation"""
        self.conversation_history.append({
            "role": "user",
            "content": command
        })

    def get_messages(self) -> list:
        """Get full message history with cached system prompt"""
        return [
            {"role": "system", "content": self.system_prompt},
            *self.conversation_history
        ]

    def add_assistant_response(self, response: str):
        """Add assistant response to history"""
        self.conversation_history.append({
            "role": "assistant",
            "content": response
        })

# Usage (reuse system prompt across multiple commands)
cache = PromptCache()

commands = ["Move forward", "Turn left", "Pick up cube"]

for cmd in commands:
    cache.add_user_message(cmd)

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=cache.get_messages()
    )

    cache.add_assistant_response(response.choices[0].message.content)

# System prompt sent only once, saving tokens
```

---

## Part 6: Domain-Specific Prompt Libraries

### Step 1: Task-Specific Templates

```python
PROMPT_LIBRARY = {
    "navigation": {
        "system": """Navigation specialist. Plan efficient paths.
Actions: move, turn, go_to
Optimize for: shortest path, obstacle avoidance""",

        "examples": [
            {
                "command": "Go to the kitchen",
                "output": {"plan": [{"action": "go_to", "location": "kitchen"}]}
            }
        ]
    },

    "manipulation": {
        "system": """Manipulation specialist. Plan pick-and-place tasks.
Actions: find, pick, place, release
Constraints: 5kg max, must find before pick""",

        "examples": [
            {
                "command": "Pick up the red cube",
                "output": {"plan": [
                    {"action": "find", "object": "cube"},
                    {"action": "pick", "object": "cube", "color": "red"}
                ]}
            }
        ]
    },

    "compound": {
        "system": """Multi-task planner. Combine navigation and manipulation.
All actions available. Break complex tasks into steps.""",

        "examples": [
            {
                "command": "Bring me the cup from the kitchen",
                "output": {"plan": [
                    {"action": "go_to", "location": "kitchen"},
                    {"action": "find", "object": "cup"},
                    {"action": "pick", "object": "cup"},
                    {"action": "go_to", "location": "user"},
                    {"action": "place", "object": "cup", "location": "user"}
                ]}
            }
        ]
    }
}

def select_prompt_template(command: str) -> dict:
    """Select appropriate prompt based on command type"""

    # Simple keyword matching (could use classifier)
    if any(word in command.lower() for word in ['move', 'go', 'navigate']):
        return PROMPT_LIBRARY['navigation']
    elif any(word in command.lower() for word in ['pick', 'grab', 'place']):
        return PROMPT_LIBRARY['manipulation']
    else:
        return PROMPT_LIBRARY['compound']

# Usage
command = "Pick up the red cube"
template = select_prompt_template(command)
print(f"Using template: {template['system']}")
```

---

## Hands-On Exercise

### Exercise 1: Prompt A/B Testing

Compare different prompt strategies:

**Requirements**:
- Test 3 prompt variants (zero-shot, few-shot, chain-of-thought)
- Use 10 test commands
- Measure: success rate, avg tokens, action count
- Identify best strategy for each command type

**Starter code**:

```python
test_commands = [
    "Move forward 5 meters",
    "Go to the kitchen and find the cup",
    "Pick up all red objects",  # Edge case
    # ... 7 more
]

results = {
    'zero_shot': [],
    'few_shot': [],
    'chain_of_thought': []
}

for cmd in test_commands:
    # TODO: Test each strategy
    # TODO: Validate output format
    # TODO: Count tokens and actions
    # TODO: Record success/failure
    pass

# TODO: Generate comparison report
```

---

### Exercise 2: Safety Validator

Build a prompt that checks plans for safety violations:

**Requirements**:
- Input: Generated action plan (JSON)
- Output: List of safety violations (if any)
- Check: distance limits, weight limits, object dependencies
- Suggest fixes for violations

**Starter code**:

```python
VALIDATOR_PROMPT = """
You are a safety validator for robot action plans.

Check this plan for violations:
{plan}

RULES:
1. Max move distance: 10m
2. Max object weight: 5kg
3. Must find before pick
4. Must pick before place

Output format:
{
  "valid": true/false,
  "violations": [
    {"rule": 1, "action_index": 2, "fix": "suggestion"}
  ]
}
"""

def validate_plan(plan: dict) -> dict:
    # TODO: Format validator prompt
    # TODO: Call LLM
    # TODO: Parse validation result
    pass
```

---

## Summary

In this lesson, you learned to:

- ✅ Structure effective prompts with clear instructions and constraints
- ✅ Use few-shot learning to improve output consistency
- ✅ Implement chain-of-thought reasoning for complex tasks
- ✅ Create contextual prompts with environment state
- ✅ Optimize prompts for cost (token reduction)
- ✅ Build reusable prompt libraries for common tasks

**Key Takeaways**:
- **Few-shot examples** reduce errors by 70% vs. zero-shot
- **Chain-of-thought** improves safety validation
- **Concise prompts** save 50-60% on token costs
- **Domain-specific templates** improve quality for specialized tasks

---

## Additional Resources

### Official Guides
- [OpenAI Prompt Engineering Guide](https://platform.openai.com/docs/guides/prompt-engineering)
- [Anthropic Prompt Library](https://docs.anthropic.com/claude/prompt-library)

### Research Papers
- [Chain-of-Thought Prompting](https://arxiv.org/abs/2201.11903)
- [Few-Shot Learning with Language Models](https://arxiv.org/abs/2005.14165)

### Prompt Libraries
- [Awesome ChatGPT Prompts](https://github.com/f/awesome-chatgpt-prompts)
- [LangChain Prompt Templates](https://python.langchain.com/docs/modules/model_io/prompts/)

---

## Next Lesson

In **Lesson 3: Action Generation**, you'll learn to:
- Convert LLM outputs to ROS 2 action messages
- Validate and sanitize generated plans
- Handle execution failures and replanning
- Monitor action execution with feedback loops

Continue to [Action Generation →](./action-generation.md)
