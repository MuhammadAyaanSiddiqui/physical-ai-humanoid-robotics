# Joint State Publishing and Control

## Introduction

This lesson covers joint_state_publisher, robot_state_publisher, and basic joint control for animating robots.

## Learning Objectives

- Publish JointState messages
- Use robot_state_publisher
- Control robot joints programmatically

## robot_state_publisher

Computes TF transforms from joint states and URDF.

## Publishing Joint States

```python
from sensor_msgs.msg import JointState

msg = JointState()
msg.header.stamp = self.get_clock().now().to_msg()
msg.name = ["joint1", "joint2"]
msg.position = [0.5, 1.0]
```

## Summary

Joint state publishing enables robot visualization and control.

## Next Steps

- [Workspace Setup](../ch4-packages/workspace-setup.md)
