---
id: 002
title: Physical AI Course Implementation Plan
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-course
branch: 001-physical-ai-course
user: ayaan
command: /sp.plan
labels: ["plan", "architecture", "docusaurus", "ros2", "isaac-sim", "education"]
links:
  spec: specs/001-physical-ai-course/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-course/plan.md
  - specs/001-physical-ai-course/research.md
  - specs/001-physical-ai-course/data-model.md
  - specs/001-physical-ai-course/quickstart.md
  - specs/001-physical-ai-course/contracts/module-structure.yaml
  - specs/001-physical-ai-course/contracts/chapter-structure.yaml
  - specs/001-physical-ai-course/contracts/lesson-structure.yaml
tests:
  - N/A (planning phase)
---

## Prompt

Create a full project plan for the book **Physical AI & Humanoid Robotics**, based on the specification.

The plan must include:

### 1. Architecture Sketch
Describe the full system architecture for the course, including:
- Simulation Stack: ROS 2 → Gazebo → Unity → NVIDIA Isaac Sim
- Physical AI Stack: Jetson Orin → RealSense → IMU → ROS 2 → Robot (Unitree)
- Vision-Language-Action Stack: Whisper → LLM Planner → ROS 2 Action Server
- Capstone Workflow: Voice Command → Planning → Navigation → Perception → Manipulation
- Book Architecture: How chapters map to modules, skills, and weekly progression

### 2. Section Structure
Define the book's structure with clear sections:
- Introduction to Physical AI & Embodied Intelligence
- ROS 2 Fundamentals (Nodes, Topics, URDF, Packages)
- Simulation: Gazebo + Unity + Sensor Modeling
- NVIDIA Isaac AI Platform (Perception, VSLAM, RL, Sim-to-Real)
- Vision-Language-Action Systems (Whisper + LLM Planning)
- Humanoid Robotics (Locomotion, Balance, Manipulation)
- Capstone: Autonomous Humanoid Robot
- Hardware & Lab Setup (Workstation, Jetson Kit, Robot Options)
- Appendices and Troubleshooting

### 3. Research Approach
Explain how information will be collected:
- Official docs (ROS 2, Isaac, Unity, Unitree)
- Academic papers on embodied intelligence
- Robotics community sources (GitHub, ROS Discourse)
- Validation through simulation experiments
- Industry references for humanoids and VLA systems

### 4. Quality Validation
Define how correctness and clarity will be ensured:
- Test all ROS 2 code examples
- Validate Gazebo & Isaac simulations
- Confirm VSLAM and perception results in Isaac
- Test Whisper → LLM → ROS 2 action pipeline
- Verify learning outcomes for each chapter
- Ensure diagrams match architecture accurately

### 5. Decisions Needing Documentation
List key choices, including:
- ROS 2 Humble vs Iron
- Gazebo Classic vs Gazebo Ignition
- Unity vs Unreal for visualization
- Isaac Sim local vs cloud GPU
- Jetson Orin Nano vs Orin NX
- Unitree G1 vs Go2 vs low-cost humanoids
- LLM selection for planning (GPT, Claude, etc.)
- GitHub Pages deployment vs alternative hosting
Include tradeoffs: cost, performance, complexity, hardware needs.

### 6. Testing Strategy
Define testing for:
- ROS 2 packages and nodes
- Gazebo physics (gravity, collisions)
- Isaac perception (object detection, SLAM drift)
- VLA pipeline accuracy (voice → plan → action)
- Humanoid locomotion (balance, gait)
- Sim-to-Real consistency
- Capstone end-to-end test

### 7. Deliverables
Specify what the final book must include:
- Full Docusaurus site
- Architecture diagrams
- ROS 2 + Isaac code snippets
- Simulation workflows
- Capstone walkthrough
- Hardware setup guides
- Deployment steps for GitHub Pages

Generate the complete plan following the structure above.

## Response snapshot

Created comprehensive implementation plan for Physical AI & Humanoid Robotics Docusaurus book:

**Architecture Sketches:**
- Full system architecture diagram (learner environment, simulation stack, physical AI stack, VLA stack, deployment architecture)
- Book content architecture: Module → Chapter → Lesson mapping with 15 chapters across 4 modules + capstone
- Docusaurus project structure with docs/, src/components/, static/ organization

**Technical Context:**
- Language/Version: Markdown (MDX), Python 3.10+, JavaScript/TypeScript (Docusaurus 3.x)
- Dependencies: Docusaurus 3.x, React 18.x, Mermaid.js, Prism/Shiki, Algolia DocSearch
- Platform: Static site (GitHub Pages/Vercel), cross-platform learner environments (Ubuntu 22.04)
- Performance: <3s load time, Lighthouse >90, WCAG 2.1 AA compliant

**Constitution Check:** ✅ All 6 principles satisfied (Hands-On First, Progressive Complexity, Accessibility, Real-World Relevance, Docusaurus-Native, Iterative Validation)

**Phase 0 Research (research.md):**
9 major technology decisions documented:
1. ROS 2 Humble Hawksbill (LTS until 2027)
2. Gazebo Fortress + Unity (dual simulation strategy)
3. Isaac Sim: Hybrid local GPU + cloud (AWS g5.2xlarge fallback)
4. Jetson Orin Nano 8GB ($499, cost-effective edge AI)
5. Unitree Go2 Edu ($2.7K) + G1 premium ($16K) + simulation-only default
6. GPT-4 Turbo (primary) + Claude 3.5 Sonnet (alternative) for LLM planning
7. GitHub Pages (free, simple) for hosting
8. WCAG 2.1 AA accessibility compliance
9. Comprehensive code example documentation standards

**Phase 1 Design Artifacts:**
- **data-model.md**: 7 entities defined (Course, Module, Chapter, Lesson, Code Example, Assessment, Hardware Configuration) with relationships, validation rules, and examples
- **contracts/**: 3 YAML schema files (module-structure, chapter-structure, lesson-structure) with OpenAPI-style validation
- **quickstart.md**: Learner onboarding guide with Ubuntu/ROS 2 setup, first "Hello ROS 2" exercise, hardware decision tree (3 paths), troubleshooting, and community resources

**Decision Matrix:** 7 major decisions with chosen options, alternatives, rationale, and tradeoffs:
- ROS 2 Humble vs Iron (chose Humble LTS)
- Gazebo Fortress + Unity vs alternatives
- Local + Cloud Isaac Sim vs single approach
- Jetson Orin Nano vs NX/AGX
- Unitree Go2 vs G1 vs budget robots
- GPT-4 vs Claude vs open-source LLMs
- GitHub Pages vs Vercel/Netlify

**Testing Strategy:** 10 test categories (ROS 2 code, Gazebo physics, Isaac perception, VLA pipeline, humanoid locomotion, sim-to-real, capstone end-to-end, documentation quality, performance, accessibility)

**Deliverables:** 8 categories including full Docusaurus site (100-150 pages, 15 chapters), architecture diagrams (Mermaid.js), 50-75 code examples, simulation workflows, capstone walkthrough, hardware guides, GitHub Pages deployment, supporting materials

**Project Structure:** Docusaurus site with docs/ (content), src/components/ (React/MDX), static/ (assets), docusaurus.config.js, sidebars.js, .github/workflows/deploy.yml

**Risks & Mitigation:** 7 risks identified (ROS 2 outdated, GPU access, Isaac licensing, dependency updates, capstone complexity, timeline, hardware costs) with likelihood, impact, and mitigation strategies

## Outcome

- ✅ Impact: Created complete implementation plan with architecture diagrams, technology research, data models, content schemas, and learner onboarding guide. Provides comprehensive roadmap for building Docusaurus-based Physical AI educational book targeting beginner/intermediate learners over 13 weeks.
- 🧪 Tests: N/A (planning document)
- 📁 Files: Created plan.md (783 lines), research.md (comprehensive tech decisions), data-model.md (7 entities), quickstart.md (onboarding guide), 3 contract schema files (module, chapter, lesson structures)
- 🔁 Next prompts: Run `/sp.tasks` to generate actionable task list from this plan. Begin Phase 2 implementation with Module 1 (ROS 2 Fundamentals) content creation.
- 🧠 Reflection: Plan successfully balances educational goals with technical feasibility. Hybrid deployment strategy (local + cloud) maximizes accessibility. Dual simulation approach (Gazebo + Unity) provides both physics accuracy and visual appeal. Comprehensive research decisions provide clear rationale for all major technology choices. Content architecture (Module → Chapter → Lesson) enables independent learning and flexible pacing.

## Evaluation notes (flywheel)

- Failure modes observed: None. All planning phases completed successfully with no blockers.
- Graders run and results (PASS/FAIL): Constitution Check - PASS (all 6 principles satisfied), Data Model Validation - PASS (7 entities with complete schemas), Research Completeness - PASS (9 major decisions documented)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): During task generation (/sp.tasks), consider breaking Module 1 content creation into smaller increments (chapter-by-chapter) to enable earlier validation cycles with target audience. Current plan groups all Module 1 chapters together, which may delay feedback loop (Principle VI: Iterative Validation).
