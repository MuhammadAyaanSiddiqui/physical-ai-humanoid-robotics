# Vision-Language-Action (VLA) Pipeline

## Learning Objectives

- Integrate voice, vision, language, and action systems
- Build end-to-end VLA pipeline: Voice → LLM → Actions → Execution
- Handle multi-modal sensor fusion
- Implement closed-loop feedback control

**Estimated Time**: 4 hours

---

## Complete VLA Architecture

```
User Voice Command
    ↓
[Whisper STT] → Transcript
    ↓
[LLM Planner] → Action Plan (JSON)
    ↓
[Object Detector] → Visual Grounding
    ↓
[Action Executor] → ROS 2 Actions
    ↓
[Robot] → Feedback
    ↓
[Monitor] → Success/Failure → Replan if needed
```

---

## Implementation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class VLAPipeline(Node):
    def __init__(self):
        super().__init__('vla_pipeline')

        # Subscribe to voice transcripts
        self.create_subscription(String, '/voice/transcript', self.voice_callback, 10)

        # Subscribe to camera feed
        self.create_subscription(Image, '/camera/image_raw', self.vision_callback, 10)

        # Publish action commands
        self.action_pub = self.create_publisher(String, '/robot/action', 10)

        self.get_logger().info('VLA Pipeline ready')

    def voice_callback(self, msg):
        """Process voice command through complete pipeline"""
        transcript = msg.data

        # 1. Call LLM planner
        plan = self.call_llm(transcript)

        # 2. Ground objects in vision
        grounded_plan = self.ground_objects(plan)

        # 3. Execute actions
        self.execute_plan(grounded_plan)

    def call_llm(self, transcript):
        """Call LLM to generate action plan"""
        # TODO: Implement LLM call
        return {"actions": []}

    def ground_objects(self, plan):
        """Map language to visual objects"""
        # TODO: Implement visual grounding
        return plan

    def execute_plan(self, plan):
        """Execute action sequence"""
        for action in plan['actions']:
            self.action_pub.publish(String(data=str(action)))

def main():
    rclpy.init()
    node = VLAPipeline()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Continue to [Object Grounding →](./object-grounding.md)
