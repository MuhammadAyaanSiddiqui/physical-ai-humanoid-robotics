---
id: 001
title: Physical AI Course Specification
stage: spec
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-course
branch: 001-physical-ai-course
user: ayaan
command: /sp.specify
labels: ["spec", "course-design", "physical-ai", "robotics", "education", "ros2", "isaac-sim"]
links:
  spec: specs/001-physical-ai-course/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-course/spec.md
  - specs/001-physical-ai-course/checklists/requirements.md
tests:
  - N/A
---

## Prompt

Create a complete specification for the course **Physical AI & Humanoid Robotics**.

### Purpose
Define the scope, learning outcomes, modules, hardware requirements, weekly structure, and capstone expectations for a robotics course focused on **embodied intelligence**, **humanoid systems**, and **AI-driven physical interaction**. This specification will guide book creation in Docusaurus.

### Focus & Theme
AI Systems in the Physical World, bridging digital intelligence with physical embodiment. Students use ROS 2, Gazebo, Unity, and NVIDIA Isaac to build and control humanoid robots in simulation and real environments.

### Course Goals
- Teach Physical AI and embodied intelligence.
- Train students to control humanoid robots using ROS 2, Gazebo, Unity, Isaac Sim, and LLM-based planning.
- Enable students to build digital twins and deploy perception, navigation, and manipulation pipelines.
- Build to a final capstone: an autonomous humanoid responding to natural-language voice commands.

### Module Breakdown
#### Module 1 — The Robotic Nervous System (ROS 2)
- ROS 2 nodes, topics, services, and actions.
- Python integration using `rclpy`.
- URDF modeling and robot description for humanoids.
- Launch files, parameters, and package development.

#### Module 2 — The Digital Twin (Gazebo & Unity)
- Physics-based simulation (collisions, gravity, dynamics).
- Gazebo setup with URDF/SDF.
- Unity for visualization and interaction.
- Sensor simulation: LiDAR, cameras, IMUs.

#### Module 3 — The AI-Robot Brain (NVIDIA Isaac™)
- Isaac Sim for photorealistic simulation and synthetic data.
- Isaac ROS for VSLAM, perception, and navigation.
- Nav2 for humanoid locomotion and path planning.
- Reinforcement learning and sim-to-real transfer.

#### Module 4 — Vision–Language–Action (VLA)
- Whisper-based Voice-to-Action.
- LLM-driven cognitive planning: convert instructions to ROS 2 tasks.
- Multi-modal perception and manipulation.
- Build toward autonomous human-interaction behaviors.

### Weekly Breakdown
**Weeks 1–2:** Foundations of Physical AI, sensors, embodied intelligence.
**Weeks 3–5:** ROS 2 fundamentals and package creation.
**Weeks 6–7:** Gazebo and Unity simulation pipelines.
**Weeks 8–10:** NVIDIA Isaac perception, training, and RL.
**Weeks 11–12:** Humanoid kinematics, locomotion, and interaction design.
**Week 13:** Conversational robotics with GPT and Whisper.

### Learning Outcomes
Students will:
- Master ROS 2 for controlling robots.
- Build and simulate humanoids using Gazebo & Unity.
- Use Isaac Sim for AI perception and navigation.
- Integrate LLMs for conversational robotics.
- Understand sim-to-real methods.
- Build an autonomous humanoid robot system.

### Assessments
- ROS 2 package development.
- Gazebo simulation project.
- Isaac-based perception pipeline.
- Final capstone: **Simulated autonomous humanoid with voice-controlled planning and manipulation**.

### Hardware Requirements
#### 1. Digital Twin Workstation (per student)
- **GPU:** RTX 4070 Ti (12GB) minimum; ideal RTX 3090/4090 (24GB).
- **CPU:** Intel i7 (13th Gen+) / Ryzen 9.
- **RAM:** 64 GB DDR5 recommended.
- **OS:** Ubuntu 22.04 LTS.
- Required for Isaac Sim, Unity, Gazebo, and VLA model training.

#### 2. Physical AI Edge Kit
- Jetson Orin Nano (8GB) or Orin NX (16GB).
- Intel RealSense D435i or D455.
- USB IMU module (BNO055).
- ReSpeaker USB microphone array.
- Used for real-world perception, inference, and interaction.

#### 3. Robot Lab Options
**Option A (Budget):** Unitree Go2 Edu quadruped (primary).
**Option B (Mini Humanoid):** Unitree G1 or Robotis OP3; lower-cost Hiwonder humanoids for kinematics only.
**Option C (Premium):** Unitree G1 Humanoid for full sim-to-real deployments.

### Cloud vs On-Prem Architecture
If RTX hardware is unavailable:
- Use AWS g5.2xlarge or g6e instances for Isaac Sim.
- Students train in cloud; deploy weights to local Jetson.
- Edge kits and at least one physical robot still required.

### Course Architecture Summary
- **Sim Rig:** RTX workstation running Isaac, Unity, Gazebo, VLA models.
- **Edge Brain:** Jetson Orin Nano for real-world inference.
- **Sensors:** RealSense camera + optional LiDAR.
- **Robot:** Unitree Go2 or G1 (shared).

### Capstone Project
Build a **Fully Autonomous Humanoid Agent** that:
1. Listens to natural-language voice commands (Whisper).
2. Uses an LLM for cognitive planning.
3. Maps and navigates the environment with VSLAM.
4. Detects and identifies objects.
5. Manipulates an object or performs a task.
6. Runs first in Isaac Sim, then (optionally) on real hardware.

Generate the full specification based on the details above.

## Response snapshot

Created comprehensive feature specification for Physical AI & Humanoid Robotics Course on branch `001-physical-ai-course`:

**User Stories (6 total)**:
1. **P1** - Foundational Physical AI Understanding: Conceptual grounding in Physical AI, sensors, embodied intelligence (Weeks 1-2)
2. **P1** - ROS 2 Robot Control Mastery: Building nodes, topics, services, URDF models, packages (Weeks 3-5)
3. **P2** - Digital Twin Simulation Environments: Gazebo and Unity simulation with physics and sensors (Weeks 6-7)
4. **P2** - AI-Powered Perception and Navigation: Isaac Sim, VSLAM, Nav2, reinforcement learning (Weeks 8-10)
5. **P3** - Voice-Controlled Cognitive Robotics: Whisper + LLM integration for multi-step commands (Weeks 11-13)
6. **P3** - Capstone Autonomous Humanoid System: Full integration of all learned skills

**Functional Requirements (22 total)**:
- Content & Curriculum (FR-001 to FR-005): 13-week structure, exercises, assessments, capstone
- Technical Integration (FR-006 to FR-011): ROS 2, Gazebo, Unity, Isaac Sim, VLA pipelines, sim-to-real
- Hardware & Environment (FR-012 to FR-015): RTX workstations, Jetson edge kits, cloud alternatives (AWS g5.2xlarge)
- Assessment & Validation (FR-016 to FR-018): Module assessments, rubrics, capstone validation
- Documentation & Accessibility (FR-019 to FR-022): Docusaurus delivery, troubleshooting, installation guides

**Success Criteria (10 measurable outcomes)**:
- 80% complete first ROS 2 node (SC-001)
- 75% launch Gazebo/Unity simulation (SC-002)
- 70% deploy Isaac perception pipeline (SC-003)
- 60% complete capstone project (SC-004)
- 85% average on module quizzes (SC-005)
- 90% error-free code examples (SC-006)
- 70% documentation search success (SC-007)
- <3s page load times (SC-008)
- 4.0/5.0 content clarity rating (SC-009)
- 80% capstone integration success rate (SC-010)

**Key Entities**: Module, Lesson, Exercise, Assessment, Code Example, Hardware Configuration, Learning Outcome

**Hardware Configurations**:
- Digital Twin Workstation: RTX 4070 Ti (12GB) minimum, 64GB RAM, Ubuntu 22.04
- Physical AI Edge Kit: Jetson Orin Nano/NX, RealSense D435i/D455, IMU, microphone array
- Robot Lab Options: Unitree Go2 ($2.7K), Unitree G1 ($16K), or Robotis OP3 ($10K)
- Cloud Alternative: AWS g5.2xlarge (~$1.20/hour)

**Validation**: Specification passed all quality checks with no [NEEDS CLARIFICATION] markers. Ready for `/sp.plan` phase.

## Outcome

- ✅ Impact: Created complete course specification defining learning journey from Physical AI foundations through autonomous humanoid capstone project. Establishes clear scope, success criteria, and hardware requirements for 13-week educational program targeting beginner/intermediate learners.
- 🧪 Tests: N/A (specification document)
- 📁 Files: Created specs/001-physical-ai-course/spec.md (333 lines) and checklists/requirements.md (validation checklist)
- 🔁 Next prompts: Run `/sp.plan` to develop technical architecture, project structure, and design artifacts for Docusaurus implementation. Consider `/sp.clarify` if additional requirements emerge during planning.
- 🧠 Reflection: Specification successfully balances educational goals with technical feasibility. Six prioritized user stories provide independent value increments. Hardware requirements offer flexible deployment options (workstation/cloud/edge). Course structure aligns with constitution principles: hands-on first, progressive complexity, real-world relevance.

## Evaluation notes (flywheel)

- Failure modes observed: None. Initial PowerShell script error was handled gracefully by manual directory creation.
- Graders run and results (PASS/FAIL): Specification Quality Checklist - PASS (all items ✅)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): During planning phase, validate whether 13-week timeline with 8-12 hours/week is realistic for beginner/intermediate learners given the technical depth (ROS 2, Isaac Sim, VLA integration). May need to adjust module pacing or scope.
