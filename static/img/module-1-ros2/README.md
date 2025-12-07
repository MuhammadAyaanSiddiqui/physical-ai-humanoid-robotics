# Module 1 ROS 2 Images

This directory contains screenshots and diagrams for Module 1: ROS 2 Robot Control Mastery.

## Required Screenshots

### RViz Screenshots

**rviz-simple-humanoid.png**
- Screenshot of RViz displaying the simple humanoid URDF model
- Should show: TF tree visualization, robot model in 3D view, joint state publisher controls
- Recommended resolution: 1920x1080
- Format: PNG
- How to capture:
  ```bash
  ros2 launch urdf_tutorial display.launch.py model:=simple_humanoid.urdf
  # Adjust view in RViz, then screenshot
  ```

**rviz-tf-tree.png**
- Screenshot of RViz TF display showing humanoid link hierarchy
- Should show: TF frames with labels, coordinate axes, parent-child relationships
- Recommended resolution: 1920x1080
- Format: PNG

**rviz-joint-controls.png**
- Screenshot of joint_state_publisher_gui showing sliders for all humanoid joints
- Should show: Joint names, slider controls, current joint values
- Recommended resolution: 800x600
- Format: PNG

### rqt_graph Screenshots

**rqt-graph-publisher-subscriber.png**
- Screenshot of rqt_graph showing publisher-subscriber node graph
- Should show: Nodes as ovals, topics as rectangles, arrows showing data flow
- Example: minimal_publisher → /topic → minimal_subscriber
- Recommended resolution: 1920x1080
- Format: PNG
- How to capture:
  ```bash
  # Terminal 1
  ros2 run demo_nodes_py talker
  # Terminal 2
  ros2 run demo_nodes_py listener
  # Terminal 3
  rqt_graph
  # Refresh graph, then screenshot
  ```

**rqt-graph-service-client.png**
- Screenshot of rqt_graph showing service client-server connections
- Should show: Service nodes, service connections (dashed lines)
- Recommended resolution: 1920x1080
- Format: PNG

**rqt-graph-multi-node.png**
- Screenshot of rqt_graph showing complex multi-node system
- Should show: 3+ nodes, multiple topics, bidirectional communication
- Recommended resolution: 1920x1080
- Format: PNG

## Diagrams Already Created

These diagrams are embedded in the documentation as Mermaid diagrams but may need PNG exports:

- ROS 2 node graph (publisher/subscriber) - Created via Mermaid in lessons
- URDF structure diagram - Created via Mermaid in lessons
- Package structure tree - Created via Mermaid in lessons

## Usage in Documentation

These images are referenced in:
- `docs/module-1-ros2/ch1-fundamentals/nodes-topics.md` - rqt_graph screenshots
- `docs/module-1-ros2/ch1-fundamentals/services-actions.md` - service graph screenshots
- `docs/module-1-ros2/ch3-urdf/rviz-visualization.md` - RViz screenshots
- `docs/module-1-ros2/ch3-urdf/joint-controllers.md` - joint control screenshots

## Screenshot Guidelines

1. **Resolution**: Use 1920x1080 for full window captures, 800x600 for GUI panels
2. **Format**: PNG with transparency where applicable
3. **Naming**: Use descriptive kebab-case names
4. **Optimization**: Compress images to <500KB using tools like TinyPNG
5. **Alt Text**: Ensure all images have descriptive alt text in markdown
6. **Dark/Light Mode**: Capture in light mode for consistency unless dark mode demonstrates specific feature

## TODO: Actual Screenshots

⚠️ **Action Required**: These screenshots need to be captured from running ROS 2 Humble environment:

- [ ] rviz-simple-humanoid.png
- [ ] rviz-tf-tree.png
- [ ] rviz-joint-controls.png
- [ ] rqt-graph-publisher-subscriber.png
- [ ] rqt-graph-service-client.png
- [ ] rqt-graph-multi-node.png

**Prerequisites for capturing**:
- Ubuntu 22.04 with ROS 2 Humble installed
- RViz2, rqt_graph, joint_state_publisher_gui packages installed
- Simple humanoid URDF model from `static/code/ros2-packages/simple_humanoid.urdf`

**Capture Instructions**:
1. Set up ROS 2 environment: `source /opt/ros/humble/setup.bash`
2. Run the necessary nodes/tools per screenshot
3. Adjust window size and view for optimal presentation
4. Use screenshot tool (e.g., `gnome-screenshot`, `flameshot`)
5. Save to this directory with exact filenames listed above
6. Optimize file sizes before committing
