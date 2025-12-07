# Action Generation and ROS 2 Conversion

## Learning Objectives

By the end of this lesson, you will be able to:

- Convert LLM JSON outputs to ROS 2 action goals
- Validate and sanitize generated action sequences
- Implement action execution with feedback monitoring
- Handle invalid or infeasible actions gracefully
- Build a complete LLM → ROS 2 action pipeline
- Optimize execution for real-time robotics

---

## Prerequisites

**Required Knowledge**:
- Completion of [Lesson 2: Prompt Engineering](./prompt-engineering.md)
- ROS 2 actions (action servers/clients)
- JSON parsing in Python

**Required Software**:
- ROS 2 Humble
- Python 3.10+
- OpenAI or Anthropic API access

**Estimated Time**: 3-4 hours

---

## Introduction

LLMs generate high-level action plans in JSON format:

```json
{
  "plan": [
    {"action": "navigate", "location": "kitchen"},
    {"action": "pick", "object": "cup", "color": "red"}
  ]
}
```

These must be converted to ROS 2 action goals and executed sequentially with error handling.

**Pipeline Overview**:
```
Voice Command → LLM → JSON Plan → Validation → ROS 2 Actions → Robot Execution
```

---

## Part 1: Action Schema Definition

### Step 1: Define Action Schema

```python
#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Optional, List
from enum import Enum

class ActionType(Enum):
    NAVIGATE = "navigate"
    PICK = "pick"
    PLACE = "place"
    SCAN = "scan"
    WAIT = "wait"

@dataclass
class NavigateAction:
    action_type: ActionType = ActionType.NAVIGATE
    location: Optional[str] = None
    x: Optional[float] = None
    y: Optional[float] = None
    theta: Optional[float] = None

@dataclass
class PickAction:
    action_type: ActionType = ActionType.PICK
    object_name: str = ""
    color: Optional[str] = None
    size: Optional[str] = None

@dataclass
class PlaceAction:
    action_type: ActionType = ActionType.PLACE
    object_name: str = ""
    location: str = ""
    x: Optional[float] = None
    y: Optional[float] = None

@dataclass
class ScanAction:
    action_type: ActionType = ActionType.SCAN
    target: Optional[str] = None

@dataclass
class WaitAction:
    action_type: ActionType = ActionType.WAIT
    duration: float = 0.0
```

---

### Step 2: JSON to Action Converter

```python
#!/usr/bin/env python3
import json
from typing import List, Union

ActionUnion = Union[NavigateAction, PickAction, PlaceAction, ScanAction, WaitAction]

class ActionConverter:
    """Convert LLM JSON output to typed action objects"""

    def parse_plan(self, llm_output: str) -> List[ActionUnion]:
        """Parse LLM JSON string into action objects"""
        try:
            data = json.loads(llm_output)
            plan = data.get('plan', [])

            actions = []
            for action_dict in plan:
                action = self.parse_action(action_dict)
                if action:
                    actions.append(action)

            return actions

        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON from LLM: {e}")

    def parse_action(self, action_dict: dict) -> Optional[ActionUnion]:
        """Parse single action dictionary"""
        action_type = action_dict.get('action', '').lower()

        if action_type == 'navigate':
            return NavigateAction(
                location=action_dict.get('location'),
                x=action_dict.get('x'),
                y=action_dict.get('y'),
                theta=action_dict.get('theta')
            )

        elif action_type == 'pick':
            return PickAction(
                object_name=action_dict.get('object', ''),
                color=action_dict.get('color'),
                size=action_dict.get('size')
            )

        elif action_type == 'place':
            return PlaceAction(
                object_name=action_dict.get('object', ''),
                location=action_dict.get('location', ''),
                x=action_dict.get('x'),
                y=action_dict.get('y')
            )

        elif action_type == 'scan':
            return ScanAction(
                target=action_dict.get('target')
            )

        elif action_type == 'wait':
            return WaitAction(
                duration=action_dict.get('duration', 0.0)
            )

        else:
            print(f"Unknown action type: {action_type}")
            return None

# Usage
converter = ActionConverter()

llm_json = '''
{
  "plan": [
    {"action": "navigate", "location": "kitchen"},
    {"action": "pick", "object": "cup", "color": "red"}
  ]
}
'''

actions = converter.parse_plan(llm_json)
for action in actions:
    print(f"{action.action_type.value}: {action}")
```

---

## Part 2: Action Validation

### Step 1: Validation Rules

```python
#!/usr/bin/env python3
from typing import List, Tuple

class ActionValidator:
    """Validate action sequences for feasibility and safety"""

    def __init__(self):
        self.max_move_distance = 10.0  # meters
        self.max_object_weight = 5.0   # kg
        self.known_locations = {'kitchen', 'bedroom', 'living_room'}
        self.object_weights = {
            'cup': 0.3,
            'plate': 0.5,
            'book': 1.0,
            'box': 2.0,
            'engine': 50.0  # Too heavy!
        }

    def validate_plan(self, actions: List[ActionUnion]) -> Tuple[bool, List[str]]:
        """
        Validate entire action plan

        Returns:
            (is_valid, list_of_errors)
        """
        errors = []

        for i, action in enumerate(actions):
            action_errors = self.validate_action(action, i, actions)
            errors.extend(action_errors)

        return (len(errors) == 0, errors)

    def validate_action(self, action: ActionUnion, index: int, all_actions: List) -> List[str]:
        """Validate single action"""
        errors = []

        if isinstance(action, NavigateAction):
            # Check location is known
            if action.location and action.location not in self.known_locations:
                errors.append(f"Action {index}: Unknown location '{action.location}'")

            # Check distance if using x/y coordinates
            if action.x is not None and action.y is not None:
                distance = (action.x**2 + action.y**2) ** 0.5
                if distance > self.max_move_distance:
                    errors.append(f"Action {index}: Movement distance {distance:.1f}m exceeds limit of {self.max_move_distance}m")

        elif isinstance(action, PickAction):
            # Check object weight
            weight = self.object_weights.get(action.object_name.lower(), 0.0)
            if weight > self.max_object_weight:
                errors.append(f"Action {index}: Object '{action.object_name}' ({weight}kg) exceeds weight limit ({self.max_object_weight}kg)")

            # Check previous actions: must scan before pick
            if not self._has_previous_scan(action.object_name, index, all_actions):
                errors.append(f"Action {index}: Must scan for '{action.object_name}' before picking")

        elif isinstance(action, PlaceAction):
            # Check we picked something before placing
            if not self._has_previous_pick(action.object_name, index, all_actions):
                errors.append(f"Action {index}: Cannot place '{action.object_name}' without picking it first")

        return errors

    def _has_previous_scan(self, object_name: str, current_index: int, actions: List) -> bool:
        """Check if object was scanned before current action"""
        for i in range(current_index):
            action = actions[i]
            if isinstance(action, ScanAction):
                if action.target is None or action.target == object_name:
                    return True
        return False

    def _has_previous_pick(self, object_name: str, current_index: int, actions: List) -> bool:
        """Check if object was picked before current action"""
        for i in range(current_index):
            action = actions[i]
            if isinstance(action, PickAction) and action.object_name == object_name:
                return True
        return False

# Usage
validator = ActionValidator()

# Valid plan
valid_actions = [
    ScanAction(target="cup"),
    PickAction(object_name="cup", color="red"),
    PlaceAction(object_name="cup", location="table")
]

is_valid, errors = validator.validate_plan(valid_actions)
print(f"Valid: {is_valid}")
if errors:
    for error in errors:
        print(f"  ❌ {error}")

# Invalid plan (missing scan)
invalid_actions = [
    PickAction(object_name="cup"),  # No scan!
]

is_valid, errors = validator.validate_plan(invalid_actions)
print(f"\nValid: {is_valid}")
for error in errors:
    print(f"  ❌ {error}")
```

---

## Part 3: ROS 2 Action Integration

### Step 1: Define ROS 2 Action Messages

Create `NavigateToGoal.action`:
```
# Goal
string action_type  # navigate, pick, place, scan
string location
float32 x
float32 y
float32 theta
string object_name
string object_color
---
# Result
bool success
string message
---
# Feedback
string current_status
float32 progress
```

---

### Step 2: Action Executor Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import NavigateToGoal
from std_msgs.msg import String
import json
import time

class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')

        # Subscribers
        self.plan_sub = self.create_subscription(
            String,
            'llm/action_plan',
            self.plan_callback,
            10
        )

        # Action clients
        self.nav_client = ActionClient(self, NavigateToGoal, 'execute_action')

        # State
        self.current_plan = []
        self.current_action_index = 0
        self.executing = False

        # Initialize validator and converter
        self.converter = ActionConverter()
        self.validator = ActionValidator()

        self.get_logger().info('Action Executor ready')

    def plan_callback(self, msg: String):
        """Receive and execute action plan"""
        try:
            # Parse JSON plan
            actions = self.converter.parse_plan(msg.data)

            # Validate plan
            is_valid, errors = self.validator.validate_plan(actions)

            if not is_valid:
                self.get_logger().error(f"Invalid plan: {errors}")
                return

            # Execute plan
            self.current_plan = actions
            self.current_action_index = 0
            self.executing = True

            self.execute_next_action()

        except Exception as e:
            self.get_logger().error(f"Plan execution failed: {e}")

    def execute_next_action(self):
        """Execute next action in plan"""
        if not self.executing:
            return

        if self.current_action_index >= len(self.current_plan):
            self.get_logger().info('✅ Plan completed successfully!')
            self.executing = False
            return

        action = self.current_plan[self.current_action_index]
        self.get_logger().info(f'Executing action {self.current_action_index + 1}/{len(self.current_plan)}: {action.action_type.value}')

        # Convert to ROS 2 action goal
        goal = self.action_to_ros_goal(action)

        # Send goal
        self.nav_client.wait_for_server()
        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    def action_to_ros_goal(self, action: ActionUnion) -> NavigateToGoal.Goal:
        """Convert action object to ROS 2 action goal"""
        goal = NavigateToGoal.Goal()

        if isinstance(action, NavigateAction):
            goal.action_type = 'navigate'
            goal.location = action.location or ''
            goal.x = action.x or 0.0
            goal.y = action.y or 0.0
            goal.theta = action.theta or 0.0

        elif isinstance(action, PickAction):
            goal.action_type = 'pick'
            goal.object_name = action.object_name
            goal.object_color = action.color or ''

        elif isinstance(action, PlaceAction):
            goal.action_type = 'place'
            goal.object_name = action.object_name
            goal.location = action.location

        elif isinstance(action, ScanAction):
            goal.action_type = 'scan'
            goal.object_name = action.target or ''

        return goal

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.executing = False
            return

        self.get_logger().info('Goal accepted')

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle action feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'  Progress: {feedback.progress:.0f}% - {feedback.current_status}')

    def result_callback(self, future):
        """Handle action result"""
        result = future.result().result

        if result.success:
            self.get_logger().info(f'  ✅ Action succeeded: {result.message}')

            # Move to next action
            self.current_action_index += 1
            self.execute_next_action()
        else:
            self.get_logger().error(f'  ❌ Action failed: {result.message}')
            self.executing = False

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutor()

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

### Step 3: Complete Pipeline Test

**Terminal 1** (LLM Planning Node):
```bash
python3 llm_planning_node.py
```

**Terminal 2** (Action Executor):
```bash
python3 action_executor.py
```

**Terminal 3** (Mock Action Server for testing):
```bash
python3 mock_action_server.py
```

**Terminal 4** (Send command):
```bash
ros2 service call /llm_plan my_robot_interfaces/srv/LLMPlan \
  "{command: 'Go to the kitchen and pick up the red cup'}"
```

**Expected flow**:
```
[llm_planning_node] Generating plan...
[llm_planning_node] Published plan with 3 actions
[action_executor] Validating plan... ✓
[action_executor] Executing action 1/3: navigate
[mock_action_server] Navigating to kitchen...
[action_executor] Progress: 50% - Moving
[action_executor] ✅ Action succeeded
[action_executor] Executing action 2/3: scan
[action_executor] Executing action 3/3: pick
[action_executor] ✅ Plan completed successfully!
```

---

## Part 4: Error Recovery

### Step 1: Action Retry Logic

```python
class RobustActionExecutor(ActionExecutor):
    """Action executor with retry and recovery"""

    def __init__(self):
        super().__init__()
        self.max_retries = 3
        self.current_retries = 0

    def result_callback(self, future):
        """Handle result with retry logic"""
        result = future.result().result

        if result.success:
            # Success - reset retries and move to next action
            self.current_retries = 0
            self.current_action_index += 1
            self.execute_next_action()

        else:
            # Failure - retry or skip
            self.current_retries += 1

            if self.current_retries < self.max_retries:
                self.get_logger().warn(
                    f'Action failed. Retrying ({self.current_retries}/{self.max_retries})...'
                )
                time.sleep(1)  # Wait before retry
                self.execute_next_action()  # Retry same action

            else:
                self.get_logger().error(
                    f'Action failed after {self.max_retries} retries. Skipping.'
                )
                self.current_retries = 0
                self.current_action_index += 1
                self.execute_next_action()  # Skip to next action
```

---

### Step 2: Replanning on Failure

```python
def result_callback_with_replanning(self, future):
    """Replan if action fails"""
    result = future.result().result

    if not result.success:
        self.get_logger().warn(f'Action failed: {result.message}')

        # Get current state
        current_state = self.get_robot_state()

        # Replan remaining tasks
        remaining_goal = self.get_remaining_goal()

        self.get_logger().info(f'Replanning to achieve: {remaining_goal}')

        # Call LLM to generate new plan
        new_plan = self.replan(remaining_goal, current_state)

        # Execute new plan
        self.current_plan = new_plan
        self.current_action_index = 0
        self.execute_next_action()

def replan(self, goal: str, state: dict) -> List[ActionUnion]:
    """Call LLM to generate new plan"""
    # TODO: Call LLM service with updated state
    # Return new action list
    pass
```

---

## Hands-On Exercise

### Exercise 1: Action Cost Estimator

Build a system to estimate execution time and cost before running plan:

**Requirements**:
- Estimate time for each action type (navigate: 5s, pick: 3s, etc.)
- Calculate total plan duration
- Warn if plan exceeds time budget
- Display progress bar during execution

**Starter code**:

```python
class ActionCostEstimator:
    def __init__(self):
        self.action_costs = {
            ActionType.NAVIGATE: 5.0,  # seconds
            ActionType.PICK: 3.0,
            ActionType.PLACE: 2.0,
            ActionType.SCAN: 2.0
        }

    def estimate_plan(self, actions: List[ActionUnion]) -> dict:
        """Estimate plan execution time and cost"""
        # TODO: Sum action costs
        # TODO: Add transition overhead
        # TODO: Calculate battery usage
        # TODO: Return estimate dict
        pass

# Usage
estimator = ActionCostEstimator()
estimate = estimator.estimate_plan(actions)
print(f"Estimated time: {estimate['duration']}s")
print(f"Battery: {estimate['battery_percent']}%")
```

---

### Exercise 2: Parallel Action Execution

Implement parallel execution for independent actions:

**Requirements**:
- Identify actions that can run in parallel (scan + navigate to different areas)
- Execute parallel actions simultaneously
- Wait for all to complete before next sequential action
- Handle partial failures

**Hint**: Use `asyncio` or ROS 2 executor threads

---

## Summary

In this lesson, you learned to:

- ✅ Convert LLM JSON outputs to typed action objects
- ✅ Validate action plans for feasibility and safety
- ✅ Implement ROS 2 action execution pipeline
- ✅ Handle action failures with retries and replanning
- ✅ Build complete LLM → ROS 2 integration

**Key Takeaways**:
- **Validation is critical**: Check feasibility before execution
- **Type safety**: Use dataclasses to catch errors early
- **Error handling**: Always retry and replan on failures
- **Feedback loops**: Monitor execution progress for debugging

---

## Additional Resources

### ROS 2 Actions
- [ROS 2 Action Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [Action Design Patterns](https://design.ros2.org/articles/actions.html)

### Validation Libraries
- [Pydantic](https://pydantic-docs.helpmanual.io/) - Data validation
- [JSON Schema](https://json-schema.org/) - Schema validation

---

## Next Lesson

In **Lesson 4: Error Handling**, you'll learn to:
- Detect and diagnose action failures
- Implement intelligent fallback strategies
- Handle ambiguous commands with clarification
- Build fault-tolerant robot systems

Continue to [Error Handling →](./error-handling.md)
