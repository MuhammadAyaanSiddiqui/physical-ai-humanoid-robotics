# Voice Command Parsing: Intent and Slot Extraction

## Learning Objectives

By the end of this lesson, you will be able to:

- Extract intent (action) and entities (slots) from voice transcripts
- Implement rule-based parsers for structured robotics commands
- Use spaCy for natural language understanding (NLU)
- Map parsed commands to ROS 2 action messages
- Handle ambiguous commands with clarification dialogs
- Build a complete voice→command→action pipeline

---

## Prerequisites

**Required Knowledge**:
- Python programming (regex, classes)
- Basic NLP concepts (tokenization, named entities)
- ROS 2 actions and services
- Completion of [Lesson 3: Speech-to-Text](./speech-to-text.md)

**Required Software**:
- Python 3.10+
- spaCy (`python3 -m spacy download en_core_web_sm`)
- ROS 2 Humble

**Estimated Time**: 3-4 hours

---

## Introduction

Raw speech transcripts like **"Move forward three meters and turn left"** must be parsed into structured commands:

```json
{
  "intent": "navigate",
  "actions": [
    {
      "type": "move",
      "direction": "forward",
      "distance": 3.0,
      "unit": "meters"
    },
    {
      "type": "turn",
      "direction": "left",
      "angle": 90.0,
      "unit": "degrees"
    }
  ]
}
```

**Command parsing challenges**:
- **Ambiguity**: "Go to the table" (which table? how?)
- **Variation**: "Move ahead 3m" vs. "Go forward three meters"
- **Compound commands**: "Pick up the cube and place it on the shelf"
- **Contextual pronouns**: "Grab it" (what is "it"?)
- **Units**: "Move 5 feet" vs. "Move 1.5 meters"

**Approaches**:

| Approach | Pros | Cons | Best For |
|----------|------|------|----------|
| **Rule-based (regex)** | Fast, deterministic, no training | Brittle, doesn't generalize | Structured commands (factories) |
| **NLU (spaCy)** | Handles variation, entities | Requires training for domain | Semi-structured (research labs) |
| **LLM (GPT-4)** | Most flexible, few-shot | Slow, expensive, unreliable | Open-ended tasks (next lesson) |

This lesson focuses on **rule-based + spaCy** for deterministic, low-latency parsing. Lesson 12 covers LLM-based parsing for complex tasks.

---

## Part 1: Rule-Based Command Parsing

### Step 1: Define Command Grammar

Start with structured command templates:

**Navigation commands**:
- `move {direction} {distance} {unit}`
- `turn {direction} [{angle} degrees]`
- `go to {location}`
- `stop`

**Manipulation commands**:
- `pick up {object}`
- `place {object} on {location}`
- `grab {object}`
- `release`

**Perception commands**:
- `look at {object}`
- `find {object}`
- `scan the area`

---

### Step 2: Regex-Based Parser

```python
#!/usr/bin/env python3
import re
from dataclasses import dataclass
from typing import List, Optional

@dataclass
class NavigationCommand:
    """Structured navigation command"""
    action: str  # move, turn, stop, go_to
    direction: Optional[str] = None  # forward, backward, left, right
    distance: Optional[float] = None
    angle: Optional[float] = None
    location: Optional[str] = None

class CommandParser:
    def __init__(self):
        # Define regex patterns for each command type
        self.patterns = {
            # Movement: "move forward 3 meters"
            'move': re.compile(
                r'move\s+(forward|backward|ahead|back)\s+(\d+\.?\d*)\s*(meters?|m|feet|ft)',
                re.IGNORECASE
            ),

            # Turn: "turn left 90 degrees" or "turn right"
            'turn': re.compile(
                r'turn\s+(left|right)(?:\s+(\d+\.?\d*)\s*degrees?)?',
                re.IGNORECASE
            ),

            # Go to: "go to the kitchen"
            'go_to': re.compile(
                r'go\s+to\s+(?:the\s+)?(\w+)',
                re.IGNORECASE
            ),

            # Stop: "stop" or "halt"
            'stop': re.compile(
                r'\b(stop|halt|freeze)\b',
                re.IGNORECASE
            )
        }

    def parse(self, transcript: str) -> Optional[NavigationCommand]:
        """Parse transcript into NavigationCommand"""

        # Try each pattern
        for action, pattern in self.patterns.items():
            match = pattern.search(transcript)
            if match:
                return self._extract_command(action, match)

        return None  # No match found

    def _extract_command(self, action: str, match) -> NavigationCommand:
        """Extract command from regex match"""

        if action == 'move':
            direction = match.group(1).lower()
            # Normalize "ahead" -> "forward", "back" -> "backward"
            direction = 'forward' if direction == 'ahead' else direction
            direction = 'backward' if direction == 'back' else direction

            distance = float(match.group(2))

            # Convert units to meters
            unit = match.group(3).lower()
            if 'feet' in unit or unit == 'ft':
                distance *= 0.3048  # feet to meters

            return NavigationCommand(
                action='move',
                direction=direction,
                distance=distance
            )

        elif action == 'turn':
            direction = match.group(1).lower()
            angle = float(match.group(2)) if match.group(2) else 90.0  # Default 90°

            return NavigationCommand(
                action='turn',
                direction=direction,
                angle=angle
            )

        elif action == 'go_to':
            location = match.group(1).lower()

            return NavigationCommand(
                action='go_to',
                location=location
            )

        elif action == 'stop':
            return NavigationCommand(action='stop')

        return None

# Usage
parser = CommandParser()

commands = [
    "Move forward 3 meters",
    "turn left 45 degrees",
    "go to the kitchen",
    "stop",
    "move ahead 5 feet"  # Converts to meters
]

for cmd in commands:
    result = parser.parse(cmd)
    print(f"'{cmd}' -> {result}")
```

**Output**:
```
'Move forward 3 meters' -> NavigationCommand(action='move', direction='forward', distance=3.0, angle=None, location=None)
'turn left 45 degrees' -> NavigationCommand(action='turn', direction='left', distance=None, angle=45.0, location=None)
'go to the kitchen' -> NavigationCommand(action='go_to', direction=None, distance=None, angle=None, location='kitchen')
'stop' -> NavigationCommand(action='stop', direction=None, distance=None, angle=None, location=None)
'move ahead 5 feet' -> NavigationCommand(action='move', direction='forward', distance=1.524, angle=None, location=None)
```

---

### Step 3: Compound Command Parsing

Handle commands with multiple actions:

```python
import re

class CompoundCommandParser:
    def __init__(self):
        self.parser = CommandParser()

        # Conjunctions that separate commands
        self.conjunctions = r'\s+(and|then|after that)\s+'

    def parse(self, transcript: str) -> List[NavigationCommand]:
        """Parse compound commands separated by 'and', 'then', etc."""

        # Split on conjunctions
        parts = re.split(self.conjunctions, transcript, flags=re.IGNORECASE)

        # Filter out conjunction words
        command_parts = [p for p in parts if p.lower() not in ['and', 'then', 'after that']]

        # Parse each part
        commands = []
        for part in command_parts:
            cmd = self.parser.parse(part)
            if cmd:
                commands.append(cmd)

        return commands

# Usage
parser = CompoundCommandParser()

transcript = "Move forward 3 meters and turn left 90 degrees"
commands = parser.parse(transcript)

for i, cmd in enumerate(commands):
    print(f"Step {i+1}: {cmd}")
```

**Output**:
```
Step 1: NavigationCommand(action='move', direction='forward', distance=3.0, ...)
Step 2: NavigationCommand(action='turn', direction='left', angle=90.0, ...)
```

---

## Part 2: NLU with spaCy

For more flexible parsing with entity recognition:

### Step 1: Install spaCy

```bash
pip3 install spacy

# Download English language model
python3 -m spacy download en_core_web_sm
```

---

### Step 2: Entity Extraction

```python
#!/usr/bin/env python3
import spacy

# Load spaCy model
nlp = spacy.load("en_core_web_sm")

def extract_entities(transcript: str):
    """Extract entities from transcript using spaCy"""
    doc = nlp(transcript)

    entities = {
        "numbers": [],
        "objects": [],
        "locations": [],
        "directions": []
    }

    # Extract numeric values
    for token in doc:
        if token.like_num:
            entities["numbers"].append(float(token.text))

    # Extract named entities
    for ent in doc.ents:
        if ent.label_ == "CARDINAL":  # Numbers
            entities["numbers"].append(ent.text)
        elif ent.label_ in ["GPE", "LOC", "FAC"]:  # Locations
            entities["locations"].append(ent.text)

    # Extract direction words (custom dictionary)
    direction_words = {"forward", "backward", "left", "right", "up", "down"}
    for token in doc:
        if token.text.lower() in direction_words:
            entities["directions"].append(token.text.lower())

    # Extract object nouns (potential manipulation targets)
    for token in doc:
        if token.pos_ == "NOUN" and token.dep_ in ["dobj", "pobj"]:
            entities["objects"].append(token.text.lower())

    return entities

# Usage
transcripts = [
    "Move forward three meters",
    "Pick up the red cube on the table",
    "Go to the kitchen and find the cup"
]

for transcript in transcripts:
    entities = extract_entities(transcript)
    print(f"\n'{transcript}'")
    print(f"  Numbers: {entities['numbers']}")
    print(f"  Objects: {entities['objects']}")
    print(f"  Locations: {entities['locations']}")
    print(f"  Directions: {entities['directions']}")
```

**Output**:
```
'Move forward three meters'
  Numbers: ['three']
  Objects: ['meters']
  Locations: []
  Directions: ['forward']

'Pick up the red cube on the table'
  Numbers: []
  Objects: ['cube', 'table']
  Locations: []
  Directions: []

'Go to the kitchen and find the cup'
  Numbers: []
  Objects: ['cup']
  Locations: []
  Directions: []
```

---

### Step 3: Intent Classification

```python
from dataclasses import dataclass
from typing import List, Dict

@dataclass
class ParsedCommand:
    intent: str  # navigate, manipulate, perceive
    entities: Dict[str, List]
    confidence: float

class IntentClassifier:
    def __init__(self):
        # Intent keywords
        self.intent_keywords = {
            'navigate': ['move', 'go', 'turn', 'walk', 'navigate', 'drive', 'stop'],
            'manipulate': ['pick', 'grab', 'place', 'put', 'drop', 'release', 'grasp'],
            'perceive': ['look', 'find', 'search', 'scan', 'detect', 'see']
        }

        self.nlp = spacy.load("en_core_web_sm")

    def classify(self, transcript: str) -> ParsedCommand:
        """Classify intent and extract entities"""
        doc = self.nlp(transcript)

        # Extract entities
        entities = self._extract_entities(doc)

        # Classify intent by keyword matching
        intent, confidence = self._classify_intent(doc)

        return ParsedCommand(
            intent=intent,
            entities=entities,
            confidence=confidence
        )

    def _extract_entities(self, doc) -> Dict[str, List]:
        """Extract all relevant entities"""
        entities = {
            "numbers": [],
            "objects": [],
            "locations": [],
            "directions": [],
            "colors": []
        }

        # Numbers
        for token in doc:
            if token.like_num or token.pos_ == "NUM":
                try:
                    entities["numbers"].append(float(token.text))
                except:
                    pass

        # Directions
        direction_words = {"forward", "backward", "left", "right", "up", "down", "ahead", "back"}
        for token in doc:
            if token.text.lower() in direction_words:
                entities["directions"].append(token.text.lower())

        # Colors
        color_words = {"red", "blue", "green", "yellow", "black", "white", "orange", "purple"}
        for token in doc:
            if token.text.lower() in color_words:
                entities["colors"].append(token.text.lower())

        # Objects (nouns that are direct/indirect objects)
        for token in doc:
            if token.pos_ == "NOUN" and token.dep_ in ["dobj", "pobj", "nsubj"]:
                # Skip location words
                if token.text.lower() not in ["kitchen", "room", "table", "shelf"]:
                    entities["objects"].append(token.text.lower())

        # Locations
        location_words = {"kitchen", "room", "table", "shelf", "floor", "counter"}
        for token in doc:
            if token.text.lower() in location_words:
                entities["locations"].append(token.text.lower())

        return entities

    def _classify_intent(self, doc) -> tuple:
        """Classify intent based on verb keywords"""
        scores = {intent: 0 for intent in self.intent_keywords}

        # Check each token against intent keywords
        for token in doc:
            for intent, keywords in self.intent_keywords.items():
                if token.lemma_.lower() in keywords:
                    scores[intent] += 1

        # Get intent with highest score
        if max(scores.values()) == 0:
            return "unknown", 0.0

        intent = max(scores, key=scores.get)
        confidence = scores[intent] / len(doc)  # Normalize by doc length

        return intent, confidence

# Usage
classifier = IntentClassifier()

commands = [
    "Move forward three meters and turn left",
    "Pick up the red cube on the table",
    "Look for the blue ball",
    "Go to the kitchen and grab the cup"
]

for cmd in commands:
    result = classifier.classify(cmd)
    print(f"\n'{cmd}'")
    print(f"  Intent: {result.intent} (confidence: {result.confidence:.2f})")
    print(f"  Entities: {result.entities}")
```

**Output**:
```
'Move forward three meters and turn left'
  Intent: navigate (confidence: 0.22)
  Entities: {'numbers': [3.0], 'directions': ['forward', 'left'], ...}

'Pick up the red cube on the table'
  Intent: manipulate (confidence: 0.14)
  Entities: {'colors': ['red'], 'objects': ['cube'], 'locations': ['table'], ...}

'Look for the blue ball'
  Intent: perceive (confidence: 0.20)
  Entities: {'colors': ['blue'], 'objects': ['ball'], ...}
```

---

## Part 3: Mapping to ROS 2 Actions

### Step 1: Define ROS 2 Action Messages

Create custom action for navigation (in `my_robot_interfaces` package):

**`NavigateToGoal.action`**:
```
# Goal
string command_type  # move, turn, go_to
float32 distance     # meters
float32 angle        # degrees
string location      # target location name
---
# Result
bool success
string message
---
# Feedback
float32 distance_remaining
float32 current_heading
```

Build:
```bash
colcon build --packages-select my_robot_interfaces
```

---

### Step 2: Command-to-Action Mapper

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import NavigateToGoal
from std_msgs.msg import String

class CommandToActionMapper(Node):
    def __init__(self):
        super().__init__('command_to_action_mapper')

        # Subscribe to parsed commands (from previous stage)
        self.cmd_sub = self.create_subscription(
            String,
            'voice/parsed_command',
            self.command_callback,
            10
        )

        # Action client for navigation
        self.nav_client = ActionClient(
            self,
            NavigateToGoal,
            'navigate_to_goal'
        )

        self.get_logger().info('Command-to-Action Mapper ready')

    def command_callback(self, msg: String):
        """Convert parsed command JSON to ROS 2 action"""
        import json

        try:
            command = json.loads(msg.data)
            self.execute_command(command)
        except Exception as e:
            self.get_logger().error(f'Invalid command: {e}')

    def execute_command(self, command: dict):
        """Execute parsed command as ROS 2 action"""
        if command['intent'] == 'navigate':
            for action in command.get('actions', []):
                self.send_navigation_goal(action)

    def send_navigation_goal(self, action: dict):
        """Send navigation goal to action server"""
        # Wait for action server
        self.nav_client.wait_for_server()

        # Create goal message
        goal_msg = NavigateToGoal.Goal()
        goal_msg.command_type = action['type']  # move, turn, go_to

        if 'distance' in action:
            goal_msg.distance = action['distance']
        if 'angle' in action:
            goal_msg.angle = action['angle']
        if 'location' in action:
            goal_msg.location = action['location']

        self.get_logger().info(f'Sending goal: {action}')

        # Send goal asynchronously
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Goal accepted, executing...')

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle action feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}m'
        )

    def result_callback(self, future):
        """Handle action result"""
        result = future.result().result

        if result.success:
            self.get_logger().info(f'Action completed: {result.message}')
        else:
            self.get_logger().error(f'Action failed: {result.message}')

def main(args=None):
    rclpy.init(args=args)
    node = CommandToActionMapper()

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

### Step 3: Complete Voice Command Pipeline

Integrate all components:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from command_parser import IntentClassifier

class VoiceCommandPipeline(Node):
    def __init__(self):
        super().__init__('voice_command_pipeline')

        # Initialize parser
        self.classifier = IntentClassifier()

        # Subscribe to transcripts (from Whisper)
        self.transcript_sub = self.create_subscription(
            String,
            'voice/transcript',
            self.transcript_callback,
            10
        )

        # Publish parsed commands
        self.command_pub = self.create_publisher(
            String,
            'voice/parsed_command',
            10
        )

        self.get_logger().info('Voice Command Pipeline started')

    def transcript_callback(self, msg: String):
        """Parse transcript and publish structured command"""
        transcript = msg.data
        self.get_logger().info(f'Transcript: {transcript}')

        # Parse with NLU
        result = self.classifier.classify(transcript)

        if result.intent == "unknown":
            self.get_logger().warn(f'Unknown command: {transcript}')
            return

        # Convert to JSON
        command_json = json.dumps({
            "intent": result.intent,
            "entities": result.entities,
            "confidence": result.confidence,
            "original_text": transcript
        })

        # Publish
        cmd_msg = String()
        cmd_msg.data = command_json
        self.command_pub.publish(cmd_msg)

        self.get_logger().info(f'Parsed: {result.intent} ({result.confidence:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandPipeline()

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

**Launch file** (`voice_pipeline.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Audio capture
        Node(
            package='voice_control',
            executable='audio_publisher.py',
            name='audio_publisher'
        ),

        # 2. Whisper transcription
        Node(
            package='voice_control',
            executable='local_whisper_node.py',
            name='whisper_node'
        ),

        # 3. Command parsing
        Node(
            package='voice_control',
            executable='voice_command_pipeline.py',
            name='command_parser'
        ),

        # 4. Command-to-action mapping
        Node(
            package='voice_control',
            executable='command_to_action_mapper.py',
            name='action_mapper'
        )
    ])
```

**Launch**:
```bash
ros2 launch voice_control voice_pipeline.launch.py
```

**Test**:
```bash
# Speak: "Move forward three meters"
# Expected ROS 2 topic flow:
# /audio/raw → /voice/transcript → /voice/parsed_command → NavigateToGoal action
```

---

## Part 4: Handling Ambiguity

### Step 1: Clarification Dialogs

```python
class AmbiguityResolver:
    def __init__(self, node):
        self.node = node

        # Publisher for clarification questions
        self.question_pub = node.create_publisher(
            String,
            'voice/clarification_question',
            10
        )

        # Subscriber for clarification responses
        self.response_sub = node.create_subscription(
            String,
            'voice/clarification_response',
            self.response_callback,
            10
        )

        self.pending_command = None

    def check_ambiguity(self, command: dict) -> bool:
        """Return True if command is ambiguous"""
        entities = command['entities']

        # Check for missing required parameters
        if command['intent'] == 'manipulate':
            if not entities.get('objects'):
                self.ask_clarification("Which object should I pick up?")
                self.pending_command = command
                return True

            if len(entities['objects']) > 1 and not entities.get('colors'):
                objects = entities['objects']
                self.ask_clarification(
                    f"I see multiple objects: {', '.join(objects)}. Which one?"
                )
                self.pending_command = command
                return True

        return False

    def ask_clarification(self, question: str):
        """Publish clarification question"""
        msg = String()
        msg.data = question
        self.question_pub.publish(msg)
        self.node.get_logger().info(f'Asking: {question}')

    def response_callback(self, msg: String):
        """Handle clarification response"""
        response = msg.data.lower()

        if self.pending_command:
            # Update command with clarification
            # (Implementation depends on command structure)
            self.node.get_logger().info(f'Clarification received: {response}')
            # ... update pending_command and execute
            self.pending_command = None

# Usage in VoiceCommandPipeline
resolver = AmbiguityResolver(self)

if resolver.check_ambiguity(command):
    return  # Wait for clarification

# Otherwise, proceed with command
self.command_pub.publish(cmd_msg)
```

---

### Step 2: Contextual Pronoun Resolution

Track previous commands to resolve "it", "there", etc.:

```python
class ContextTracker:
    def __init__(self):
        self.last_object = None
        self.last_location = None

    def update(self, command: dict):
        """Update context from executed command"""
        entities = command['entities']

        if entities.get('objects'):
            self.last_object = entities['objects'][0]

        if entities.get('locations'):
            self.last_location = entities['locations'][0]

    def resolve_pronouns(self, transcript: str) -> str:
        """Replace pronouns with contextual references"""
        # Replace "it" with last object
        if "it" in transcript.lower() and self.last_object:
            transcript = transcript.replace("it", self.last_object)

        # Replace "there" with last location
        if "there" in transcript.lower() and self.last_location:
            transcript = transcript.replace("there", self.last_location)

        return transcript

# Usage
context = ContextTracker()

# First command: "Pick up the cube"
result = classifier.classify("Pick up the cube")
context.update({"entities": result.entities})

# Second command: "Place it on the table"
transcript2 = "Place it on the table"
resolved = context.resolve_pronouns(transcript2)  # "Place cube on the table"
result2 = classifier.classify(resolved)
```

---

## Hands-On Exercise

### Exercise 1: Manipulation Command Parser

Extend the parser to handle manipulation commands:

**Requirements**:
- Parse "pick up `{color}` `{object}`" (e.g., "pick up the red cube")
- Parse "place `{object}` on `{location}`"
- Extract object attributes (color, size: small/large)
- Handle compound commands: "Pick up the cube and place it on the shelf"

**Starter code**:

```python
@dataclass
class ManipulationCommand:
    action: str  # pick_up, place, release
    object: Optional[str] = None
    color: Optional[str] = None
    size: Optional[str] = None
    location: Optional[str] = None

class ManipulationParser:
    def __init__(self):
        self.nlp = spacy.load("en_core_web_sm")

    def parse(self, transcript: str) -> Optional[ManipulationCommand]:
        # TODO: Extract intent (pick_up, place, release)
        # TODO: Extract object name
        # TODO: Extract color attribute
        # TODO: Extract location (for "place" commands)
        pass

# Test cases
test_cases = [
    "Pick up the red cube",
    "Place the ball on the table",
    "Grab the small green box",
    "Put the cup on the counter and release it"
]
```

---

### Exercise 2: Confidence-Based Fallback

Implement a system that falls back to LLM (GPT-4) when confidence is low:

**Requirements**:
- If spaCy confidence < 0.3, use GPT-4 API (from next lesson)
- Compare parsing results from both methods
- Log which method was used and accuracy

**Starter code**:

```python
class HybridParser:
    def __init__(self):
        self.local_parser = IntentClassifier()
        # TODO: Initialize OpenAI client for GPT-4

    def parse(self, transcript: str) -> dict:
        # Try local parser first
        result = self.local_parser.classify(transcript)

        if result.confidence < 0.3:
            # TODO: Fall back to GPT-4
            # TODO: Use prompt: "Parse this robot command into JSON: {transcript}"
            pass

        return result
```

---

## Summary

In this lesson, you learned to:

- ✅ Extract intent and entities from voice transcripts using regex and spaCy
- ✅ Implement rule-based parsers for structured robotics commands
- ✅ Classify intent (navigate, manipulate, perceive) with keyword matching
- ✅ Map parsed commands to ROS 2 action messages
- ✅ Handle ambiguity with clarification dialogs
- ✅ Resolve contextual pronouns using command history

**Key Takeaways**:
- **Rule-based parsers**: Fast, deterministic, good for factories/warehouses
- **spaCy NLU**: Handles linguistic variation, extracts entities
- **Hybrid approach**: Use local parsing for speed, LLM for complex cases (next lesson)
- **Ambiguity**: Always validate required parameters before executing
- **Context**: Track previous commands to resolve pronouns

---

## Additional Resources

### Official Documentation
- [spaCy Documentation](https://spacy.io/usage)
- [spaCy Entity Recognition](https://spacy.io/usage/linguistic-features#named-entities)
- [ROS 2 Actions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)

### NLU Training
- [spaCy Training Pipelines](https://spacy.io/usage/training)
- [Rasa NLU](https://rasa.com/docs/rasa/nlu-training-data/) - Alternative NLU framework

### Research Papers
- [Slot Filling and Intent Detection](https://arxiv.org/abs/1902.10909)
- [Contextual Language Understanding for Robotics](https://arxiv.org/abs/2004.14354)

---

## Next Lesson

In **Chapter 12: LLM Planning**, you'll learn to:
- Use GPT-4/Claude for complex command understanding
- Engineer prompts for robotics-specific tasks
- Generate structured JSON actions from free-form commands
- Handle multi-step planning and error recovery

Continue to [LLM Integration →](../ch12-llm-planning/llm-integration.md)
