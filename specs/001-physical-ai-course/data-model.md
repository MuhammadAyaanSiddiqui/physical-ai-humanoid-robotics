# Data Model: Physical AI & Humanoid Robotics Course Content

**Feature**: Physical AI & Humanoid Robotics Course
**Branch**: 001-physical-ai-course
**Date**: 2025-12-05
**Purpose**: Define content entities, relationships, and validation rules for course structure

## Overview

This document defines the information architecture for the Physical AI & Humanoid Robotics educational book. The model supports a hierarchical course structure (Module → Chapter → Lesson) with supporting entities for code examples, assessments, and hardware configurations.

## Entity Relationship Diagram

```
┌─────────────┐
│   Course    │
│             │
└──────┬──────┘
       │ 1:N
       ▼
┌─────────────┐       1:N      ┌──────────────┐
│   Module    │◄────────────────┤  Assessment  │
│             │                 │              │
└──────┬──────┘                 └──────────────┘
       │ 1:N
       ▼
┌─────────────┐
│   Chapter   │
│             │
└──────┬──────┘
       │ 1:N
       ▼
┌─────────────┐       M:N      ┌────────────────┐
│   Lesson    │◄───────────────►│  Code Example  │
│             │                 │                │
└─────────────┘                 └────────────────┘

┌──────────────────────┐
│ Hardware Config      │ (referenced by lessons)
│                      │
└──────────────────────┘
```

## Core Entities

### 1. Course

**Purpose**: Top-level container for entire educational program

**Properties**:
- `id` (string, required): Unique identifier (e.g., "physical-ai-humanoid-robotics")
- `title` (string, required): Full course title
- `version` (string, required): Semantic version (e.g., "1.0.0")
- `target_audience` (string, required): Audience description (e.g., "beginner to intermediate")
- `duration_weeks` (integer, required): Total course duration in weeks
- `prerequisites` (array[string], required): Required prior knowledge
- `learning_outcomes` (array[string], required): High-level course outcomes

**Relationships**:
- Has Many: Modules (1:N)

**Example**:
```yaml
id: physical-ai-humanoid-robotics
title: Physical AI & Humanoid Robotics
version: 1.0.0
target_audience: Beginners to intermediate learners with basic Python knowledge
duration_weeks: 13
prerequisites:
  - Basic Python programming (variables, functions, loops, classes)
  - Linux command-line familiarity (bash, file navigation)
  - Basic understanding of coordinate systems (2D/3D)
learning_outcomes:
  - Master ROS 2 for controlling robots
  - Build and simulate humanoids using Gazebo & Unity
  - Use NVIDIA Isaac Sim for AI perception and navigation
  - Integrate LLMs for conversational robotics
  - Understand sim-to-real methods
  - Build an autonomous humanoid robot system
```

---

### 2. Module

**Purpose**: Major topic area spanning multiple weeks (e.g., "The Robotic Nervous System (ROS 2)")

**Properties**:
- `id` (string, required): Unique module identifier (e.g., "module-1-ros2")
- `title` (string, required): Module title
- `subtitle` (string, optional): Descriptive subtitle
- `weeks` (integer, required): Duration in weeks
- `week_range` (string, required): Week numbers covered (e.g., "3-5")
- `description` (text, required): Module overview
- `learning_outcomes` (array[string], required): Module-specific outcomes
- `prerequisites` (array[string], required): Prior modules or external knowledge
- `order` (integer, required): Sequential position in course

**Relationships**:
- Belongs To: Course (N:1)
- Has Many: Chapters (1:N)
- Has Many: Assessments (1:N)

**Validation Rules**:
- `weeks` must be >= 1
- `order` must be unique within course
- `learning_outcomes` must have at least 3 items

**Example**:
```yaml
id: module-1-ros2
title: The Robotic Nervous System
subtitle: ROS 2 Fundamentals
weeks: 3
week_range: "3-5"
description: Master ROS 2, the middleware that powers modern robotics. Learn nodes, topics, services, URDF modeling, and package development.
learning_outcomes:
  - Create ROS 2 nodes using Python (rclpy)
  - Implement publisher/subscriber communication patterns
  - Build URDF models for humanoid robots
  - Develop ROS 2 packages with launch files and parameters
prerequisites:
  - Python basics (from course prerequisites)
  - Ubuntu 22.04 installed
  - ROS 2 Humble installed
order: 1
```

---

### 3. Chapter

**Purpose**: Major subdivision within a module, focusing on specific skill area (e.g., "ROS 2 Fundamentals")

**Properties**:
- `id` (string, required): Unique chapter identifier (e.g., "ch1-fundamentals")
- `module_id` (string, required): Parent module ID
- `title` (string, required): Chapter title
- `estimated_time` (string, required): Time to complete (e.g., "2-3 hours")
- `difficulty_level` (enum, required): "beginner" | "intermediate" | "advanced"
- `description` (text, required): Chapter overview
- `order` (integer, required): Sequential position within module

**Relationships**:
- Belongs To: Module (N:1)
- Has Many: Lessons (1:N)

**Validation Rules**:
- `module_id` must reference existing module
- `order` must be unique within module
- `estimated_time` must match pattern: "\d+-?\d* hours?"

**Example**:
```yaml
id: ch1-fundamentals
module_id: module-1-ros2
title: ROS 2 Fundamentals
estimated_time: 3-4 hours
difficulty_level: beginner
description: Introduction to ROS 2 architecture, installation, and basic concepts including nodes, topics, services, and the ROS 2 CLI.
order: 1
```

---

### 4. Lesson

**Purpose**: Single learning unit covering specific concept or skill (e.g., "Creating Your First Node")

**Properties**:
- `id` (string, required): Unique lesson identifier (e.g., "first-node")
- `chapter_id` (string, required): Parent chapter ID
- `title` (string, required): Lesson title
- `content_type` (enum, required): "theory" | "hands-on" | "exercise" | "assessment"
- `estimated_time` (string, required): Time to complete (e.g., "30 minutes")
- `prerequisites` (array[string], required): Prior lessons or knowledge
- `learning_objectives` (array[string], required): What learner will achieve
- `content_sections` (array[ContentSection], required): Structured content blocks
- `order` (integer, required): Sequential position within chapter

**Content Section Structure**:
```typescript
ContentSection {
  type: "intro" | "concept" | "example" | "exercise" | "summary"
  markdown_content: string
  code_examples?: array[CodeExampleReference]
}
```

**Relationships**:
- Belongs To: Chapter (N:1)
- References Many: Code Examples (M:N)

**Validation Rules**:
- `chapter_id` must reference existing chapter
- `learning_objectives` must have at least 2 items
- `content_sections` must include at least one "intro" and one "summary" type
- `order` must be unique within chapter

**Example**:
```yaml
id: first-node
chapter_id: ch2-python-rclpy
title: Creating Your First Node
content_type: hands-on
estimated_time: 45 minutes
prerequisites:
  - ROS 2 installed and sourced
  - Python 3.10+ available
learning_objectives:
  - Write a minimal ROS 2 node in Python using rclpy
  - Understand node lifecycle (initialization, spin, shutdown)
  - Run and verify node execution
content_sections:
  - type: intro
    markdown_content: |
      In this lesson, we'll create your first ROS 2 node...
  - type: concept
    markdown_content: |
      A ROS 2 node is a process that performs computation...
  - type: example
    markdown_content: |
      Here's a complete example of a minimal node...
    code_examples: ["minimal_node_py"]
  - type: exercise
    markdown_content: |
      Exercise: Modify the node to publish at 2 Hz instead of 1 Hz...
  - type: summary
    markdown_content: |
      You've learned how to create, run, and modify ROS 2 nodes...
order: 1
```

---

### 5. Code Example

**Purpose**: Complete, runnable code snippet demonstrating a concept

**Properties**:
- `id` (string, required): Unique example identifier (e.g., "minimal_node_py")
- `filename` (string, required): Suggested filename (e.g., "minimal_node.py")
- `language` (string, required): Programming language (e.g., "python", "cpp", "yaml")
- `title` (string, required): Example title
- `description` (text, required): What the example demonstrates
- `code` (text, required): Complete source code
- `dependencies` (array[string], required): Required packages/libraries
- `expected_output` (text, optional): Sample output when run
- `how_to_run` (text, required): Step-by-step execution instructions
- `troubleshooting` (array[Troubleshooting], optional): Common issues and solutions

**Troubleshooting Structure**:
```typescript
Troubleshooting {
  issue: string (description of problem)
  solution: string (how to fix it)
}
```

**Relationships**:
- Referenced By: Lessons (M:N)

**Validation Rules**:
- `code` must not be empty
- `dependencies` must list all non-standard libraries used in `code`
- `language` must be valid syntax highlighting language

**Example**:
```yaml
id: minimal_node_py
filename: minimal_node.py
language: python
title: Minimal ROS 2 Publisher Node
description: A basic ROS 2 node that publishes "Hello ROS 2" messages to a topic at 1 Hz.
code: |
  #!/usr/bin/env python3
  import rclpy
  from rclpy.node import Node
  from std_msgs.msg import String

  class MinimalPublisher(Node):
      def __init__(self):
          super().__init__('minimal_publisher')
          self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
          self.timer = self.create_timer(1.0, self.timer_callback)
          self.i = 0

      def timer_callback(self):
          msg = String()
          msg.data = f'Hello ROS 2: {self.i}'
          self.publisher_.publish(msg)
          self.get_logger().info(f'Publishing: "{msg.data}"')
          self.i += 1

  def main(args=None):
      rclpy.init(args=args)
      node = MinimalPublisher()
      rclpy.spin(node)
      node.destroy_node()
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
dependencies:
  - rclpy (ROS 2 Python client library)
  - std_msgs (ROS 2 standard message types)
expected_output: |
  [INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 0"
  [INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 1"
  [INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 2"
  ...
how_to_run: |
  1. Make the file executable:
     chmod +x minimal_node.py

  2. Source ROS 2:
     source /opt/ros/humble/setup.bash

  3. Run the node:
     python3 minimal_node.py

  4. In another terminal, echo the topic:
     source /opt/ros/humble/setup.bash
     ros2 topic echo /hello_topic
troubleshooting:
  - issue: "ModuleNotFoundError: No module named 'rclpy'"
    solution: "Ensure ROS 2 is sourced: source /opt/ros/humble/setup.bash"
  - issue: "Node not publishing (no output)"
    solution: "Check that the node is running (ps aux | grep minimal_node) and verify topic exists (ros2 topic list)"
```

---

### 6. Assessment

**Purpose**: Graded evaluation of learner understanding (project, quiz, demo)

**Properties**:
- `id` (string, required): Unique assessment identifier (e.g., "ros2-package-assessment")
- `module_id` (string, required): Parent module ID
- `title` (string, required): Assessment title
- `type` (enum, required): "project" | "quiz" | "demo" | "written"
- `estimated_time` (string, required): Time to complete (e.g., "4-6 hours")
- `description` (text, required): Assessment overview and goals
- `requirements` (array[string], required): Specific deliverables/criteria
- `rubric` (array[RubricItem], required): Grading criteria
- `example_solution` (text, optional): Reference solution or approach
- `submission_format` (string, required): How to submit (e.g., "GitHub repo link", "Video demo")

**Rubric Item Structure**:
```typescript
RubricItem {
  criterion: string (what is being evaluated)
  points: integer (max points for this criterion)
  description: string (how to achieve full points)
}
```

**Relationships**:
- Belongs To: Module (N:1)

**Validation Rules**:
- `module_id` must reference existing module
- `rubric` total points should equal 100 (by convention)
- `requirements` must have at least 3 items

**Example**:
```yaml
id: ros2-package-assessment
module_id: module-1-ros2
title: "Assessment: Build a Multi-Node ROS 2 Package"
type: project
estimated_time: 4-6 hours
description: |
  Create a ROS 2 package with multiple nodes that communicate via topics and services. The package should model a simple sensor-actuator system.
requirements:
  - Create a ROS 2 package using colcon with proper package.xml and setup.py
  - Implement a "sensor" node that publishes simulated data (e.g., temperature) at 2 Hz
  - Implement an "actuator" node that subscribes to sensor data and logs alerts when thresholds are exceeded
  - Implement a "controller" service that allows changing the alert threshold
  - Include launch file that starts all nodes
  - Include README with setup and usage instructions
rubric:
  - criterion: Package structure and build system
    points: 15
    description: Package builds with colcon, proper manifest files, follows ROS 2 conventions
  - criterion: Sensor node functionality
    points: 20
    description: Publishes data at correct rate, uses appropriate message type, logs output
  - criterion: Actuator node functionality
    points: 20
    description: Subscribes to sensor topic, implements threshold logic, logs alerts
  - criterion: Controller service
    points: 20
    description: Service server correctly receives and applies threshold changes
  - criterion: Launch file
    points: 10
    description: Launch file starts all nodes with correct parameters
  - criterion: Code quality
    points: 10
    description: Clear variable names, comments for complex logic, proper error handling
  - criterion: Documentation
    points: 5
    description: README explains setup, usage, and provides examples
example_solution: |
  (Instructor provides reference implementation or key design decisions)
submission_format: GitHub repository link with README
```

---

### 7. Hardware Configuration

**Purpose**: Specification for physical hardware setups (workstation, edge kit, robot)

**Properties**:
- `id` (string, required): Unique config identifier (e.g., "digital-twin-workstation")
- `name` (string, required): Configuration name
- `purpose` (string, required): What this configuration enables
- `components` (array[Component], required): List of hardware components
- `total_cost` (number, required): Estimated total cost in USD
- `difficulty` (enum, required): "easy" | "moderate" | "advanced"
- `setup_guide_url` (string, optional): Link to setup documentation

**Component Structure**:
```typescript
Component {
  name: string (component name)
  spec: string (technical specification)
  cost: number (estimated cost in USD)
  required: boolean (true if mandatory, false if optional)
  alternatives?: array[Alternative] (other options)
}

Alternative {
  name: string
  spec: string
  cost: number
  tradeoffs: string
}
```

**Relationships**:
- Referenced By: Lessons (via setup instructions)

**Example**:
```yaml
id: digital-twin-workstation
name: Digital Twin Workstation
purpose: Run Isaac Sim, Unity, Gazebo, and train VLA models locally
components:
  - name: GPU
    spec: NVIDIA RTX 4070 Ti (12GB VRAM)
    cost: 800
    required: true
    alternatives:
      - name: RTX 3090
        spec: 24GB VRAM
        cost: 1200
        tradeoffs: More VRAM for larger models, but more expensive
      - name: RTX 4090
        spec: 24GB VRAM
        cost: 1600
        tradeoffs: Best performance, highest cost
  - name: CPU
    spec: Intel i7 13th Gen or AMD Ryzen 9 7900X
    cost: 400
    required: true
  - name: RAM
    spec: 64GB DDR5
    cost: 200
    required: true
    alternatives:
      - name: 32GB DDR5
        spec: Minimum viable
        cost: 100
        tradeoffs: May struggle with large Isaac Sim scenes
  - name: Storage
    spec: 500GB NVMe SSD
    cost: 60
    required: true
  - name: OS
    spec: Ubuntu 22.04 LTS
    cost: 0
    required: true
total_cost: 1460
difficulty: moderate
setup_guide_url: /docs/appendix-a-hardware/workstation-setup
```

---

## Entity Relationships Summary

```
Course (1)
  ├─ has_many → Modules (4)
  │    ├─ has_many → Chapters (15)
  │    │    └─ has_many → Lessons (75+)
  │    │         └─ references_many → Code Examples (50-75)
  │    └─ has_many → Assessments (6)
  └─ references → Hardware Configurations (3)
```

## Validation Rules Summary

| Entity | Required Fields | Unique Constraints | Referential Integrity |
|--------|----------------|--------------------|-----------------------|
| Course | id, title, version, duration_weeks | id | N/A |
| Module | id, title, weeks, outcomes, order | id, order (within course) | course_id exists |
| Chapter | id, module_id, title, order | id, order (within module) | module_id exists |
| Lesson | id, chapter_id, title, objectives, order | id, order (within chapter) | chapter_id exists |
| Code Example | id, filename, code, dependencies | id | N/A |
| Assessment | id, module_id, rubric | id | module_id exists |
| Hardware Config | id, components, total_cost | id | N/A |

## Content Lifecycle

```
Draft → Review → Approved → Published → Updated → Archived
```

- **Draft**: Initial content creation, may have incomplete sections
- **Review**: Content under peer/expert review
- **Approved**: Content meets quality standards, ready for publication
- **Published**: Live on production site, accessible to learners
- **Updated**: Published content revised based on feedback or dependency changes
- **Archived**: Content deprecated, maintained for historical reference

## Metadata Standards

All content files include frontmatter:

```markdown
---
id: unique-identifier
title: Human-Readable Title
sidebar_label: Short Label
sidebar_position: 1
tags: [ros2, python, beginner]
last_updated: 2025-12-05
status: published
---
```

---

**Status**: Complete - Ready for contracts/ generation
**Next Steps**: Generate YAML schema files in `contracts/` directory
