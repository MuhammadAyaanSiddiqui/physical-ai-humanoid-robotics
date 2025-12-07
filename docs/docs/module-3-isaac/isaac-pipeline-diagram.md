# Isaac Sim → Isaac ROS → Nav2 Pipeline

## System Architecture Diagram

```mermaid
graph TB
    subgraph "Isaac Sim (Simulation)"
        IS[Isaac Sim Scene]
        Robot[Simulated Robot]
        Sensors[Virtual Sensors<br/>Camera, LiDAR, IMU]
        IS --> Robot
        Robot --> Sensors
    end

    subgraph "ROS 2 Bridge"
        Bridge[ROS 2 Bridge]
        Sensors --> Bridge
    end

    subgraph "Isaac ROS (Perception)"
        VSLAM[Visual SLAM<br/>Mapping + Localization]
        Detection[Object Detection<br/>YOLO]
        Depth[Depth Estimation<br/>Stereo/Mono]
        Pose[6DOF Pose<br/>Estimation]

        Bridge --> VSLAM
        Bridge --> Detection
        Bridge --> Depth
        Bridge --> Pose
    end

    subgraph "Nav2 (Navigation)"
        Planner[Path Planner<br/>A*, Smac]
        Controller[Local Planner<br/>DWA, TEB]
        Costmap[Costmap Layers<br/>Static, Obstacle, Inflation]
        BT[Behavior Tree<br/>Navigation Logic]

        VSLAM --> Costmap
        Detection --> Costmap
        Costmap --> Planner
        Planner --> Controller
        BT --> Planner
        BT --> Controller
    end

    subgraph "Robot Control"
        Motors[Motor Commands<br/>Velocity/Torque]
        Controller --> Motors
    end

    Motors --> Robot

    style IS fill:#e1f5ff
    style VSLAM fill:#fff4e1
    style Detection fill:#fff4e1
    style Planner fill:#e8f5e9
    style Controller fill:#e8f5e9
    style Motors fill:#fce4ec
```

## Data Flow

### Perception Pipeline

```mermaid
sequenceDiagram
    participant Sim as Isaac Sim
    participant Bridge as ROS 2 Bridge
    participant VSLAM as Visual SLAM
    participant Det as Object Detection
    participant Nav as Nav2

    Sim->>Bridge: Camera Images (30 Hz)
    Sim->>Bridge: LiDAR Scans (10 Hz)
    Sim->>Bridge: IMU Data (100 Hz)

    Bridge->>VSLAM: Stereo Images
    VSLAM->>VSLAM: Build Map + Localize
    VSLAM->>Nav: Robot Pose (odom)

    Bridge->>Det: RGB Image
    Det->>Det: YOLO Inference
    Det->>Nav: Detected Objects

    Nav->>Nav: Update Costmap
    Nav->>Nav: Plan Path
    Nav->>Sim: Velocity Commands
```

## Component Interaction

### Isaac Sim → Isaac ROS

| Isaac Sim Output | ROS 2 Topic | Isaac ROS Consumer | Purpose |
|------------------|-------------|-------------------|---------|
| RGB Camera | `/camera/image_raw` | Object Detection, VSLAM | Vision input |
| Stereo Camera | `/stereo/left`, `/stereo/right` | Stereo Depth | 3D perception |
| LiDAR Scan | `/scan` | Obstacle Layer | Collision avoidance |
| IMU | `/imu/data` | VSLAM, Odometry | Motion estimation |
| Depth Image | `/camera/depth` | Depth Estimation | 3D mapping |

### Isaac ROS → Nav2

| Isaac ROS Output | Nav2 Consumer | Purpose |
|------------------|---------------|---------|
| `/visual_slam/tracking/odometry` | Localization | Robot pose |
| `/detections` | Semantic Layer | Object-aware planning |
| `/point_cloud` | Costmap | 3D obstacle detection |
| `/depth_image` | Costmap | Obstacle detection |

### Nav2 → Robot Control

| Nav2 Output | Robot Actuator | Purpose |
|-------------|----------------|---------|
| `/cmd_vel` (Twist) | Wheel Motors | Differential drive |
| `/joint_commands` | Joint Controllers | Humanoid locomotion |

## Performance Characteristics

```mermaid
gantt
    title Processing Latency (ms)
    dateFormat X
    axisFormat %L

    section Perception
    Camera Capture           :0, 33
    VSLAM Update            :33, 50
    Object Detection        :33, 20

    section Planning
    Costmap Update          :50, 200
    Global Planning         :200, 50
    Local Planning          :250, 50

    section Control
    Command Generation      :300, 10
    Motor Actuation         :310, 20
```

**Total Latency**: ~330ms (camera → motor commands)
**Target**: &lt;500ms for safe navigation

## Scalability

### Parallel Processing

```
Isaac Sim: 1 Scene
    ↓
Isaac ROS: 4 Perception Nodes (parallel on GPU)
    ↓
Nav2: 3 Planners (sequential)
    ↓
Robot: 1 Controller
```

**Throughput**: 30 FPS end-to-end

---

This diagram illustrates the complete data flow from simulation through perception to autonomous navigation in the Isaac ecosystem.
