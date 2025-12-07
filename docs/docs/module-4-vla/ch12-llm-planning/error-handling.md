# Error Handling and Fault Recovery

## Learning Objectives

By the end of this lesson, you will be able to:

- Detect and classify LLM and robot execution errors
- Implement retry strategies with exponential backoff
- Build clarification dialogues for ambiguous commands
- Design fallback strategies for failed actions
- Create fault-tolerant robot systems
- Monitor and log errors for debugging

---

## Prerequisites

**Required Knowledge**:
- Completion of [Lesson 3: Action Generation](./action-generation.md)
- Exception handling in Python
- ROS 2 action feedback mechanisms

**Estimated Time**: 2-3 hours

---

## Introduction

Robust robot systems must handle failures gracefully:

**Error Categories**:
1. **LLM Errors**: Invalid JSON, hallucinated actions, rate limits
2. **Validation Errors**: Infeasible plans, constraint violations
3. **Execution Errors**: Action failures, hardware faults, obstacles
4. **Ambiguity Errors**: Underspecified commands, missing information

**Error Handling Strategy**:
```
Error Detection → Classification → Recovery Action → Logging
```

---

## Part 1: LLM Error Handling

### Step 1: JSON Parsing Errors

```python
#!/usr/bin/env python3
import json
import re
from typing import Optional

class LLMOutputParser:
    """Robust parser for LLM outputs"""

    def parse_json(self, llm_output: str, max_retries: int = 3) -> Optional[dict]:
        """Parse JSON with fallback strategies"""

        # Strategy 1: Direct JSON parsing
        try:
            return json.loads(llm_output)
        except json.JSONDecodeError:
            pass

        # Strategy 2: Extract JSON from markdown code blocks
        json_match = re.search(r'```(?:json)?\s*(\{.*?\})\s*```', llm_output, re.DOTALL)
        if json_match:
            try:
                return json.loads(json_match.group(1))
            except json.JSONDecodeError:
                pass

        # Strategy 3: Find first { } pair
        start = llm_output.find('{')
        end = llm_output.rfind('}')
        if start != -1 and end != -1 and end > start:
            try:
                return json.loads(llm_output[start:end+1])
            except json.JSONDecodeError:
                pass

        # Strategy 4: Fix common JSON issues
        fixed = self.fix_common_json_errors(llm_output)
        if fixed:
            try:
                return json.loads(fixed)
            except json.JSONDecodeError:
                pass

        # All strategies failed
        return None

    def fix_common_json_errors(self, text: str) -> Optional[str]:
        """Fix common JSON formatting errors"""

        # Remove trailing commas
        text = re.sub(r',(\s*[}\]])', r'\1', text)

        # Fix single quotes to double quotes
        text = text.replace("'", '"')

        # Fix unquoted keys (simple cases)
        text = re.sub(r'(\{|,)\s*([a-zA-Z_][a-zA-Z0-9_]*)\s*:', r'\1 "\2":', text)

        return text

# Usage
parser = LLMOutputParser()

# Malformed JSON from LLM
bad_output = """
Here's the plan:
{
  plan: [
    {"action": "move", "distance": 3.0,}  // trailing comma
  ]
}
"""

result = parser.parse_json(bad_output)
if result:
    print("✅ Successfully parsed:", result)
else:
    print("❌ Failed to parse JSON")
```

---

### Step 2: Hallucination Detection

```python
class HallucinationDetector:
    """Detect when LLM invents non-existent actions or capabilities"""

    def __init__(self):
        # Define valid action set
        self.valid_actions = {
            'navigate', 'pick', 'place', 'scan', 'wait'
        }

        # Define known entities
        self.known_objects = {
            'cup', 'plate', 'cube', 'ball', 'book', 'box'
        }

        self.known_locations = {
            'kitchen', 'bedroom', 'living_room', 'table', 'shelf'
        }

    def detect_hallucinations(self, plan: dict) -> list:
        """Detect hallucinated actions or entities"""
        hallucinations = []

        for i, action in enumerate(plan.get('plan', [])):
            # Check action type
            action_type = action.get('action', '').lower()
            if action_type not in self.valid_actions:
                hallucinations.append({
                    'type': 'invalid_action',
                    'action_index': i,
                    'value': action_type,
                    'message': f"Action '{action_type}' does not exist"
                })

            # Check object names
            if 'object' in action:
                obj = action['object'].lower()
                if obj not in self.known_objects:
                    hallucinations.append({
                        'type': 'unknown_object',
                        'action_index': i,
                        'value': obj,
                        'message': f"Object '{obj}' not in environment"
                    })

            # Check locations
            if 'location' in action:
                loc = action['location'].lower()
                if loc not in self.known_locations:
                    hallucinations.append({
                        'type': 'unknown_location',
                        'action_index': i,
                        'value': loc,
                        'message': f"Location '{loc}' not found"
                    })

        return hallucinations

# Usage
detector = HallucinationDetector()

# Plan with hallucinated action
bad_plan = {
    'plan': [
        {'action': 'fly', 'height': 10.0},  # Robot can't fly!
        {'action': 'pick', 'object': 'dragon'}  # Dragon doesn't exist!
    ]
}

hallucinations = detector.detect_hallucinations(bad_plan)
for h in hallucinations:
    print(f"❌ {h['type']}: {h['message']}")
```

---

### Step 3: Rate Limit Handling

```python
import time
from openai import OpenAI, RateLimitError
from collections import deque

class RateLimitedLLMClient:
    """LLM client with automatic rate limiting"""

    def __init__(self, requests_per_minute: int = 50):
        self.client = OpenAI()
        self.rpm = requests_per_minute
        self.request_times = deque()

    def call_llm(self, messages: list, max_retries: int = 5) -> str:
        """Call LLM with rate limiting and retries"""

        for attempt in range(max_retries):
            try:
                # Wait if rate limit would be exceeded
                self._wait_if_rate_limited()

                # Make request
                response = self.client.chat.completions.create(
                    model="gpt-4o-mini",
                    messages=messages,
                    temperature=0.0
                )

                # Log request time
                self.request_times.append(time.time())

                return response.choices[0].message.content

            except RateLimitError:
                wait_time = 2 ** attempt  # Exponential backoff
                print(f"⚠️ Rate limit hit. Waiting {wait_time}s... (attempt {attempt+1}/{max_retries})")
                time.sleep(wait_time)

        raise Exception("Max retries exceeded due to rate limiting")

    def _wait_if_rate_limited(self):
        """Wait if we're about to exceed rate limit"""
        now = time.time()

        # Remove requests older than 1 minute
        while self.request_times and self.request_times[0] < now - 60:
            self.request_times.popleft()

        # Check if at limit
        if len(self.request_times) >= self.rpm:
            # Wait until oldest request is 60s old
            sleep_time = 60 - (now - self.request_times[0])
            if sleep_time > 0:
                print(f"⏳ Rate limit approaching. Waiting {sleep_time:.1f}s...")
                time.sleep(sleep_time)

# Usage
client = RateLimitedLLMClient(requests_per_minute=50)

# Make 100 requests - will automatically throttle
for i in range(100):
    response = client.call_llm([
        {"role": "user", "content": f"Plan {i}"}
    ])
```

---

## Part 2: Execution Error Handling

### Step 1: Action Failure Recovery

```python
from enum import Enum

class FailureReason(Enum):
    OBSTACLE = "obstacle_detected"
    OBJECT_NOT_FOUND = "object_not_found"
    GRASP_FAILED = "grasp_failed"
    TIMEOUT = "timeout"
    HARDWARE_ERROR = "hardware_error"
    UNKNOWN = "unknown"

class ActionRecoveryStrategy:
    """Determine recovery action based on failure reason"""

    def get_recovery_plan(self, failed_action: dict, reason: FailureReason) -> list:
        """Generate recovery plan based on failure type"""

        if reason == FailureReason.OBSTACLE:
            return [
                {'action': 'navigate', 'x': -0.5, 'y': 0.0},  # Back up
                {'action': 'scan'},  # Re-scan environment
                failed_action  # Retry original action
            ]

        elif reason == FailureReason.OBJECT_NOT_FOUND:
            return [
                {'action': 'scan', 'target': failed_action.get('object')},
                {'action': 'wait', 'duration': 1.0},
                failed_action  # Retry after scanning
            ]

        elif reason == FailureReason.GRASP_FAILED:
            return [
                {'action': 'release'},  # Open gripper
                {'action': 'wait', 'duration': 0.5},
                failed_action  # Retry grasp
            ]

        elif reason == FailureReason.TIMEOUT:
            # Simplify action (e.g., break into smaller steps)
            return self.simplify_action(failed_action)

        else:
            # Unknown error - ask for help
            return [
                {'action': 'wait', 'duration': 1.0},
                {'action': 'request_help', 'reason': str(reason)}
            ]

    def simplify_action(self, action: dict) -> list:
        """Break complex action into simpler steps"""

        if action['action'] == 'navigate' and 'distance' in action:
            # Break long movement into smaller steps
            distance = action['distance']
            if distance > 2.0:
                return [
                    {'action': 'navigate', 'direction': action['direction'], 'distance': 2.0},
                    {'action': 'navigate', 'direction': action['direction'], 'distance': distance - 2.0}
                ]

        return [action]  # Can't simplify, return as-is

# Usage
recovery = ActionRecoveryStrategy()

failed_action = {'action': 'pick', 'object': 'cup'}
reason = FailureReason.OBJECT_NOT_FOUND

recovery_plan = recovery.get_recovery_plan(failed_action, reason)
print(f"Recovery plan: {recovery_plan}")
```

---

### Step 2: Replanning with LLM

```python
class AdaptivePlanner:
    """Use LLM to replan when execution fails"""

    def __init__(self, llm_client):
        self.client = llm_client

    def replan_after_failure(
        self,
        original_command: str,
        failed_action: dict,
        failure_reason: str,
        current_state: dict
    ) -> dict:
        """Ask LLM to generate alternative plan"""

        replan_prompt = f"""
ORIGINAL COMMAND: {original_command}

EXECUTION FAILURE:
- Failed action: {failed_action}
- Reason: {failure_reason}

CURRENT STATE:
- Location: {current_state.get('location')}
- Holding: {current_state.get('holding') or 'nothing'}
- Visible objects: {current_state.get('visible_objects', [])}

Generate an alternative plan to achieve the original goal, avoiding the failed action.
Consider:
1. Can we achieve the goal differently?
2. Do we need to gather more information first?
3. Is the goal still achievable given the failure?

Output JSON plan or {{" error": "goal_not_achievable", "reason": "..." }} if impossible.
"""

        response = self.client.call_llm([
            {"role": "system", "content": "You are a robot replanning assistant."},
            {"role": "user", "content": replan_prompt}
        ])

        return json.loads(response)

# Usage
planner = AdaptivePlanner(client)

alternative_plan = planner.replan_after_failure(
    original_command="Pick up the red cup",
    failed_action={'action': 'pick', 'object': 'cup'},
    failure_reason="Cup is too heavy (6kg > 5kg limit)",
    current_state={'location': 'kitchen', 'holding': None, 'visible_objects': ['cup', 'plate']}
)

print(json.dumps(alternative_plan, indent=2))
```

---

## Part 3: Clarification Dialogues

### Step 1: Ambiguity Detection

```python
class AmbiguityDetector:
    """Detect when commands are underspecified"""

    def detect_ambiguity(self, command: str, environment: dict) -> Optional[str]:
        """Return clarification question if command is ambiguous"""

        # Check for vague references
        if 'it' in command.lower():
            return "What does 'it' refer to?"

        if 'there' in command.lower():
            return "Where is 'there'?"

        # Check for unspecified objects
        if 'pick' in command.lower() or 'grab' in command.lower():
            # Check if object is specified
            objects_in_view = environment.get('visible_objects', [])

            # If multiple objects and none specified
            if len(objects_in_view) > 1 and not any(obj in command.lower() for obj in objects_in_view):
                return f"Which object? I see: {', '.join(objects_in_view)}"

        # Check for unspecified colors when multiple same objects
        object_counts = self._count_objects_by_type(environment.get('visible_objects', []))
        for obj_type, count in object_counts.items():
            if count > 1 and obj_type in command.lower():
                return f"Which {obj_type}? I see {count} of them."

        return None  # No ambiguity detected

    def _count_objects_by_type(self, objects: list) -> dict:
        """Count objects by type"""
        counts = {}
        for obj in objects:
            obj_type = obj.split('_')[0]  # 'red_cup' → 'cup'
            counts[obj_type] = counts.get(obj_type, 0) + 1
        return counts

# Usage
detector = AmbiguityDetector()

env = {'visible_objects': ['red_cup', 'blue_cup', 'plate']}

command = "Pick up the cup"
clarification = detector.detect_ambiguity(command, env)

if clarification:
    print(f"🤔 {clarification}")
```

---

### Step 2: Interactive Clarification

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ClarificationNode(Node):
    """ROS 2 node for handling clarifications"""

    def __init__(self):
        super().__init__('clarification_node')

        # Publishers
        self.question_pub = self.create_publisher(
            String, 'voice/clarification_question', 10
        )

        # Subscribers
        self.response_sub = self.create_subscription(
            String, 'voice/clarification_response', self.response_callback, 10
        )

        self.pending_command = None
        self.clarification_context = {}

    def ask_clarification(self, question: str, original_command: str, context: dict):
        """Ask user for clarification"""
        self.pending_command = original_command
        self.clarification_context = context

        msg = String()
        msg.data = question
        self.question_pub.publish(msg)

        self.get_logger().info(f"🤔 Asking: {question}")

    def response_callback(self, msg: String):
        """Handle clarification response"""
        response = msg.data

        if not self.pending_command:
            return

        # Incorporate clarification into original command
        enhanced_command = f"{self.pending_command}. Specifically: {response}"

        self.get_logger().info(f"✓ Clarified: {enhanced_command}")

        # Re-process enhanced command
        # TODO: Send to LLM planning service

        # Clear pending state
        self.pending_command = None
        self.clarification_context = {}

# Usage
# User says: "Pick up the cup"
# System detects ambiguity (2 cups visible)
# System asks: "Which cup? I see: red_cup, blue_cup"
# User responds: "The red one"
# System processes: "Pick up the cup. Specifically: The red one"
```

---

## Part 4: Logging and Monitoring

### Step 1: Comprehensive Error Logging

```python
import logging
import json
from datetime import datetime

class RobotErrorLogger:
    """Structured logging for robot errors"""

    def __init__(self, log_file: str = "robot_errors.jsonl"):
        self.log_file = log_file

        # Configure Python logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s [%(levelname)s] %(message)s'
        )
        self.logger = logging.getLogger(__name__)

    def log_llm_error(self, command: str, error: Exception, llm_output: str = ""):
        """Log LLM-related errors"""
        error_record = {
            'timestamp': datetime.now().isoformat(),
            'error_type': 'llm_error',
            'command': command,
            'exception': str(error),
            'llm_output': llm_output[:500]  # Truncate long outputs
        }

        self._write_log(error_record)

    def log_execution_error(
        self,
        action: dict,
        failure_reason: str,
        recovery_attempted: bool = False
    ):
        """Log action execution errors"""
        error_record = {
            'timestamp': datetime.now().isoformat(),
            'error_type': 'execution_error',
            'action': action,
            'failure_reason': failure_reason,
            'recovery_attempted': recovery_attempted
        }

        self._write_log(error_record)

    def log_clarification_request(self, command: str, question: str, response: str = ""):
        """Log clarification interactions"""
        error_record = {
            'timestamp': datetime.now().isoformat(),
            'error_type': 'clarification_needed',
            'command': command,
            'question': question,
            'user_response': response
        }

        self._write_log(error_record)

    def _write_log(self, record: dict):
        """Write JSON record to log file"""
        with open(self.log_file, 'a') as f:
            f.write(json.dumps(record) + '\n')

        self.logger.info(json.dumps(record))

# Usage
error_logger = RobotErrorLogger()

try:
    # Attempt LLM call
    response = client.call_llm(messages)
except Exception as e:
    error_logger.log_llm_error(
        command="Pick up the cup",
        error=e,
        llm_output=""
    )
```

---

### Step 2: Error Analytics Dashboard

```python
class ErrorAnalytics:
    """Analyze error logs for patterns"""

    def __init__(self, log_file: str = "robot_errors.jsonl"):
        self.log_file = log_file

    def get_error_summary(self, hours: int = 24) -> dict:
        """Get error statistics for last N hours"""
        import json
        from datetime import datetime, timedelta

        cutoff_time = datetime.now() - timedelta(hours=hours)

        errors = {
            'llm_error': 0,
            'execution_error': 0,
            'clarification_needed': 0
        }

        failure_reasons = {}

        with open(self.log_file, 'r') as f:
            for line in f:
                record = json.loads(line)
                timestamp = datetime.fromisoformat(record['timestamp'])

                if timestamp >= cutoff_time:
                    error_type = record['error_type']
                    errors[error_type] = errors.get(error_type, 0) + 1

                    if error_type == 'execution_error':
                        reason = record.get('failure_reason', 'unknown')
                        failure_reasons[reason] = failure_reasons.get(reason, 0) + 1

        return {
            'period_hours': hours,
            'total_errors': sum(errors.values()),
            'by_type': errors,
            'top_failure_reasons': sorted(
                failure_reasons.items(),
                key=lambda x: x[1],
                reverse=True
            )[:5]
        }

# Usage
analytics = ErrorAnalytics()
summary = analytics.get_error_summary(hours=24)

print(f"Last 24 hours: {summary['total_errors']} errors")
print(f"  LLM errors: {summary['by_type']['llm_error']}")
print(f"  Execution errors: {summary['by_type']['execution_error']}")
print(f"  Clarifications: {summary['by_type']['clarification_needed']}")
print("\nTop failure reasons:")
for reason, count in summary['top_failure_reasons']:
    print(f"  {reason}: {count}")
```

---

## Hands-On Exercise

### Exercise 1: Fallback Chain

Implement a multi-level fallback strategy:

**Requirements**:
- Level 1: Retry action (3 attempts)
- Level 2: Try alternative action from recovery strategy
- Level 3: Replan with LLM
- Level 4: Ask user for help

**Starter code**:

```python
class FallbackChain:
    def execute_with_fallbacks(self, action: dict) -> bool:
        # Level 1: Retry
        for i in range(3):
            if self.try_execute(action):
                return True
            time.sleep(1)

        # Level 2: Recovery strategy
        # TODO: Get recovery plan and try executing

        # Level 3: Replan with LLM
        # TODO: Call LLM replanner

        # Level 4: Ask user
        # TODO: Publish help request

        return False
```

---

## Summary

In this lesson, you learned to:

- ✅ Handle LLM errors (JSON parsing, hallucinations, rate limits)
- ✅ Implement execution error recovery with retries and replanning
- ✅ Detect and resolve ambiguous commands through clarification
- ✅ Build comprehensive error logging and analytics
- ✅ Create fault-tolerant robot systems with fallback strategies

**Key Takeaways**:
- **Always validate**: Check LLM outputs before execution
- **Graceful degradation**: Implement fallback chains
- **Learn from failures**: Log and analyze error patterns
- **Involve humans**: Ask for clarification when uncertain

---

## Additional Resources

### Error Handling Patterns
- [Retry Pattern](https://docs.microsoft.com/en-us/azure/architecture/patterns/retry)
- [Circuit Breaker Pattern](https://martinfowler.com/bliki/CircuitBreaker.html)

### Logging Best Practices
- [Structured Logging](https://www.loggly.com/ultimate-guide/python-logging-basics/)
- [OpenTelemetry](https://opentelemetry.io/) - Observability framework

---

## Module 4 Summary

Congratulations! You've completed Chapter 12: Cognitive Planning. You can now:
- Integrate GPT-4/Claude for robot planning
- Engineer effective prompts for robotics
- Generate and execute ROS 2 actions from LLM outputs
- Handle errors and implement fault recovery

**Next**: Continue to Chapter 13 for humanoid kinematics and locomotion control.

Continue to [Chapter 13: Humanoid Control →](../ch13-humanoid-control/forward-inverse-kinematics.md)
