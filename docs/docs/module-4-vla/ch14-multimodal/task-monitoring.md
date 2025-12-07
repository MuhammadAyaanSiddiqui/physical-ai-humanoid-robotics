# Task Monitoring and Execution Feedback

## Learning Objectives

- Monitor action execution in real-time
- Detect failures early through sensor feedback
- Implement replanning when actions fail
- Provide user feedback on progress

**Estimated Time**: 3 hours

---

## Execution Monitor

```python
class TaskMonitor(Node):
    def __init__(self):
        super().__init__('task_monitor')

        # Subscribe to action feedback
        self.create_subscription(ActionFeedback, '/action/feedback', self.feedback_callback, 10)

        # Publish status updates
        self.status_pub = self.create_publisher(String, '/task/status', 10)

        self.current_action = None
        self.start_time = None

    def feedback_callback(self, msg):
        """Monitor action progress"""
        progress = msg.progress

        # Check for timeout
        if time.time() - self.start_time > 30.0:
            self.get_logger().warn("Action timeout! Requesting replan")
            self.request_replan()

        # Publish status
        status = String()
        status.data = f"Progress: {progress}%"
        self.status_pub.publish(status)

    def request_replan(self):
        """Trigger replanning when action fails"""
        # TODO: Call LLM to generate alternative plan
        pass
```

Continue to [Human-Robot Interaction →](./human-robot-interaction.md)
