# ROS 2 Code Examples

This directory contains complete, runnable ROS 2 examples for the Physical AI & Humanoid Robotics course.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble installed
- Python 3.10+

## Quick Start

### 1. Source ROS 2

```bash
source /opt/ros/humble/setup.bash
```

### 2. Run Publisher-Subscriber Example

**Terminal 1 - Publisher**:
```bash
python3 minimal_publisher.py
```

**Terminal 2 - Subscriber**:
```bash
python3 minimal_subscriber.py
```

Expected output:
- Publisher: `Publishing: "Hello World: 0"`, `Publishing: "Hello World: 1"`, ...
- Subscriber: `I heard: "Hello World: 0"`, `I heard: "Hello World: 1"`, ...

### 3. Run Service Example

**Terminal 1 - Service Server**:
```bash
python3 service_example.py server
```

**Terminal 2 - Service Client**:
```bash
python3 service_example.py client 5 3
```

Expected output:
- Server: `Incoming request: a=5, b=3 → sum=8`
- Client: `Result: 5 + 3 = 8`

### 4. Visualize URDF Model

Install required packages:
```bash
sudo apt install ros-humble-urdf-tutorial ros-humble-joint-state-publisher-gui
```

Launch visualization:
```bash
ros2 launch urdf_tutorial display.launch.py model:=$(pwd)/simple_humanoid.urdf
```

This opens RViz with the humanoid model and a GUI to control joints.

## File Descriptions

| File | Description | Topics/Services |
|------|-------------|-----------------|
| `minimal_publisher.py` | Simple publisher sending strings every 0.5s | `/chatter` (String) |
| `minimal_subscriber.py` | Simple subscriber receiving strings | `/chatter` (String) |
| `service_example.py` | Add two integers service (server + client) | `/add_two_ints` (AddTwoInts) |
| `simple_humanoid.urdf` | 10-joint humanoid robot model | N/A (model file) |
| `my_robot_package/` | Complete ROS 2 package template | Multiple |

## Troubleshooting

### "ros2: command not found"

Make sure to source ROS 2:
```bash
source /opt/ros/humble/setup.bash
```

Add to `~/.bashrc` for automatic sourcing:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### "No module named 'rclpy'"

Install ROS 2 Python client library:
```bash
sudo apt install ros-humble-rclpy
```

### "No module named 'example_interfaces'"

Install example interfaces package:
```bash
sudo apt install ros-humble-example-interfaces
```

### URDF Won't Load

Check URDF syntax:
```bash
check_urdf simple_humanoid.urdf
```

## Next Steps

1. Create your own ROS 2 package using `my_robot_package/` as a template
2. Modify the publisher/subscriber to use different message types
3. Add more joints to the humanoid URDF model
4. Experiment with service types beyond AddTwoInts

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [rclpy API Reference](https://docs.ros.org/en/humble/p/rclpy/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [ROS 2 Examples](https://github.com/ros2/examples)

## License

MIT License - Part of the Physical AI & Humanoid Robotics Course
