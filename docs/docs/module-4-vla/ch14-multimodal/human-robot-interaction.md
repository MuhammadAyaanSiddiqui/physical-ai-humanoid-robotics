# Human-Robot Interaction Patterns

## Learning Objectives

- Implement conversational interaction patterns
- Handle clarification dialogues
- Provide natural language feedback
- Build trust through transparent communication

**Estimated Time**: 2-3 hours

---

## Conversational Patterns

```python
class ConversationalRobot:
    def __init__(self):
        self.context = []

    def greet_user(self):
        return "Hello! I'm ready to help. What would you like me to do?"

    def confirm_understanding(self, parsed_command):
        return f"Just to confirm: you want me to {parsed_command}. Is that correct?"

    def report_progress(self, action, progress):
        if progress < 30:
            return f"Starting to {action}..."
        elif progress < 70:
            return f"Halfway through {action}..."
        else:
            return f"Almost done with {action}..."

    def report_success(self, action):
        return f"✓ Successfully completed {action}!"

    def report_failure(self, action, reason):
        return f"Sorry, I couldn't {action} because {reason}. Would you like me to try something else?"

    def ask_clarification(self, ambiguity):
        return f"I'm not sure about {ambiguity}. Could you please clarify?"

# Usage
robot = ConversationalRobot()

print(robot.greet_user())
print(robot.confirm_understanding("pick up the red cup"))
print(robot.report_progress("picking up cup", 50))
print(robot.report_success("pick up the cup"))
```

---

## Summary

**Module 4 (VLA) Complete!** You can now:

- ✅ Capture audio with ReSpeaker
- ✅ Transcribe speech with Whisper (cloud + local)
- ✅ Parse commands with NLU
- ✅ Plan with LLMs (GPT-4/Claude)
- ✅ Execute actions via ROS 2
- ✅ Control humanoid kinematics and locomotion
- ✅ Integrate vision-language-action pipeline

**Next**: Module 5 covers the Capstone Project where you'll integrate all systems into an autonomous humanoid robot.
