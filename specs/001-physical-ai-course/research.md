# Research & Technology Decisions: Physical AI & Humanoid Robotics Course

**Feature**: Physical AI & Humanoid Robotics Course
**Branch**: 001-physical-ai-course
**Date**: 2025-12-05
**Purpose**: Document all technology selections, rationales, and alternatives evaluated during Phase 0 planning

## Executive Summary

This document captures research findings and decisions for building a Docusaurus-based educational book on Physical AI & Humanoid Robotics. All decisions prioritize learner accessibility, hands-on learning, and alignment with industry-standard tools while balancing cost, complexity, and educational effectiveness.

## Decision 1: ROS 2 Version Selection

### Research Question
Which ROS 2 distribution should the course use as its foundation?

### Options Evaluated

1. **ROS 2 Humble Hawksbill (LTS)**
   - Release Date: May 2022
   - Support: LTS until May 2027 (5 years)
   - Ubuntu: 22.04 LTS (Jammy Jellyfish)
   - Status: Stable, widely adopted

2. **ROS 2 Iron Irwini**
   - Release Date: May 2023
   - Support: Standard (until November 2024 - already EOL)
   - Ubuntu: 22.04 LTS
   - Status: Latest features but shorter support

3. **ROS 2 Jazzy Jalisco (LTS)**
   - Release Date: May 2024
   - Support: LTS until May 2029
   - Ubuntu: 24.04 LTS (Noble Numbat)
   - Status: Newest LTS but less mature ecosystem

### Decision: ROS 2 Humble Hawksbill (LTS)

### Rationale

1. **Long-Term Support**: Humble has LTS support until May 2027, providing stability for a multi-year educational course.
2. **Isaac ROS Compatibility**: NVIDIA Isaac ROS officially supports Humble with tested packages and examples.
3. **Community Maturity**: Largest community adoption for ROS 2, extensive tutorials, troubleshooting resources, and package ecosystem.
4. **Ubuntu 22.04 LTS**: Aligns with widely-used LTS Ubuntu version, easier for learners to install and maintain.
5. **Educational Stability**: Avoids breaking changes during 13-week course, allowing consistent content.

### Tradeoffs Accepted

- **Newer Features Unavailable**: Iron and Jazzy have newer features (improved performance, additional middleware options), but these are not critical for educational content.
- **Not Bleeding Edge**: Advanced users may prefer latest releases, but stability trumps novelty for beginners.

### References

- [ROS 2 Releases](https://docs.ros.org/en/humble/Releases.html)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/getting_started/index.html)

---

## Decision 2: Simulation Platform

### Research Question
Which simulation platform(s) should the course use for robot simulation and physics?

### Options Evaluated

1. **Gazebo Classic (11)**
   - Pros: Mature, extensive plugin ecosystem, stable ROS integration
   - Cons: End-of-life (EOL), legacy architecture, performance limitations

2. **Gazebo Fortress/Garden (Ignition → New Gazebo)**
   - Pros: Modern architecture, better ROS 2 integration (`ros_gz_bridge`), improved performance, active development
   - Cons: Smaller plugin ecosystem than Classic, some community tutorials still use Classic

3. **NVIDIA Isaac Sim**
   - Pros: Photorealistic rendering, synthetic data generation, RL training, RTX acceleration
   - Cons: Requires RTX GPU, higher complexity, not suitable for all learners

4. **Unity with ROS 2 Integration**
   - Pros: High-fidelity visualization, excellent graphics, familiar to game developers
   - Cons: Complex setup, commercial licensing concerns, steeper learning curve for robotics

5. **Webots**
   - Pros: Cross-platform, easy setup, good for education
   - Cons: Smaller community than Gazebo, less standard in industry

### Decision: Dual Strategy - Gazebo Fortress (Primary) + Unity (Visualization)

### Rationale

1. **Gazebo Fortress (Primary for Physics)**:
   - Modern ROS 2 integration via `ros_gz` bridge packages
   - Active development ensures long-term viability
   - Industry-standard for robotics simulation
   - Sufficient performance for educational use on standard hardware

2. **Unity (Secondary for High-Fidelity Visualization)**:
   - Provides visual appeal for learner engagement
   - Demonstrates alternative visualization approaches
   - Useful for presentation and demo scenarios
   - Unity Robotics Hub provides ROS 2 TCP connector

3. **Isaac Sim (Module 3 Specialty)**:
   - Used specifically for Module 3 (AI-Robot Brain) where photorealism and synthetic data are essential
   - Not required for basic ROS 2 and simulation concepts
   - Cloud deployment option (AWS) addresses hardware accessibility

### Tradeoffs Accepted

- **Dual Setup Complexity**: Learners must install both Gazebo and Unity, increasing setup burden.
- **Mitigation**: Clear separation of use cases (Gazebo = physics/testing, Unity = visualization) and optional Unity path.

### References

- [Gazebo Documentation](https://gazebosim.org/docs/fortress/getstarted)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

---

## Decision 3: Isaac Sim Deployment Strategy

### Research Question
How should learners access NVIDIA Isaac Sim: local GPU, cloud, or both?

### Options Evaluated

1. **Local GPU Only (RTX 4070 Ti+ minimum)**
   - Pros: Best performance, no ongoing costs, offline capable
   - Cons: High upfront cost ($800+ for GPU), excludes learners without hardware

2. **Cloud Only (AWS g5.2xlarge with A10G GPU)**
   - Pros: Accessible to all learners, no upfront hardware cost, scalable
   - Cons: Ongoing costs (~$1.20/hour = $50-150/week), requires internet, latency for interactive workflows

3. **Hybrid Approach (Document Both Paths)**
   - Pros: Maximizes accessibility, learners choose based on resources
   - Cons: Increases documentation complexity, must maintain two sets of instructions

### Decision: Hybrid Approach - Local GPU (Primary) + Cloud (Fallback)

### Rationale

1. **Accessibility**: Not all learners can afford RTX 4070 Ti+ hardware; cloud provides entry point.
2. **Cost Transparency**: Documented AWS costs allow learners to budget (estimate $100-200 for Module 3 if intensive).
3. **Performance**: Local GPU provides best experience for those who have it.
4. **Flexibility**: Learners can start with cloud, then migrate to local if they acquire hardware.

### Implementation Details

**Local GPU Path**:
- Hardware: RTX 4070 Ti (12GB VRAM) minimum, RTX 3090/4090 (24GB) recommended
- OS: Ubuntu 22.04 native or dual-boot (WSL2 not recommended for Isaac Sim)
- Installation: NVIDIA drivers, Isaac Sim via Omniverse Launcher

**Cloud Path**:
- Instance: AWS g5.2xlarge (NVIDIA A10G, 24GB GPU, 32GB RAM, 8 vCPUs)
- Cost: ~$1.20/hour on-demand (~$36 for 30 hours of Module 3 work)
- Setup: EC2 instance with Ubuntu 22.04, Isaac Sim Docker container or direct install
- Access: SSH + X11 forwarding or remote desktop (NoMachine, VNC)

**Hybrid Workflow**:
- Train policies in cloud Isaac Sim
- Export trained weights (.onnx, .pth)
- Deploy to local Jetson Orin for edge inference

### Tradeoffs Accepted

- **Documentation Burden**: Must maintain parallel instructions for local and cloud setups.
- **Cost Uncertainty**: AWS pricing may change; learners must monitor costs.
- **Mitigation**: Clear budget guidelines, cost estimation calculator, spot instance recommendations.

### References

- [Isaac Sim System Requirements](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/requirements.html)
- [AWS g5 Instances](https://aws.amazon.com/ec2/instance-types/g5/)

---

## Decision 4: Edge Compute Hardware

### Research Question
Which Jetson module should the course recommend for edge AI deployment?

### Options Evaluated

1. **Jetson Orin Nano 8GB ($499)**
   - GPU: 1024-core NVIDIA Ampere, 32 Tensor Cores
   - RAM: 8GB LPDDR5
   - AI Performance: 40 TOPS (INT8)
   - Pros: Most cost-effective Orin option, sufficient for perception workloads
   - Cons: Limited VRAM may constrain large models

2. **Jetson Orin NX 16GB ($899)**
   - GPU: 1024-core NVIDIA Ampere, 32 Tensor Cores
   - RAM: 16GB LPDDR5
   - AI Performance: 100 TOPS (INT8)
   - Pros: More VRAM for complex models, higher throughput
   - Cons: Higher cost, may be overkill for educational use

3. **Jetson AGX Orin 32GB ($1999)**
   - GPU: 2048-core NVIDIA Ampere, 64 Tensor Cores
   - RAM: 32GB LPDDR5
   - AI Performance: 275 TOPS (INT8)
   - Pros: Highest performance, suitable for research
   - Cons: Expensive for education, excessive for beginner/intermediate learners

4. **Raspberry Pi 4/5 ($55-80)**
   - Pros: Very low cost, accessible
   - Cons: No CUDA support, insufficient for real-time perception, cannot run Isaac models

### Decision: Jetson Orin Nano 8GB (Primary Recommendation)

### Rationale

1. **Cost-Effectiveness**: $499 is accessible for education labs and motivated individuals; significantly cheaper than NX/AGX.
2. **Sufficient Performance**: 40 TOPS handles VSLAM, object detection, and Nav2 at real-time rates (15-30 FPS).
3. **Isaac ROS Compatibility**: Officially supported by NVIDIA Isaac ROS with optimized packages.
4. **Educational Sweet Spot**: Balances performance and cost for beginner/intermediate learners; not overkill.
5. **Upgrade Path**: Learners can start with Orin Nano, upgrade to NX/AGX if pursuing research.

### Optional Upgrade: Jetson Orin NX 16GB

Recommend NX for learners who:
- Plan to run larger perception models (YOLOv8-X, CLIP, SegFormer)
- Need higher throughput for multi-camera setups
- Have budget for premium hardware

### Tradeoffs Accepted

- **VRAM Limitation**: 8GB may limit largest models; mitigation via model quantization (INT8, FP16) and model selection guidance.
- **Not Future-Proof**: NX/AGX have longer viability for cutting-edge models, but Orin Nano sufficient for course scope.

### References

- [Jetson Orin Nano Specs](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
- [Isaac ROS Hardware Requirements](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/index.html)

---

## Decision 5: Robot Platform Selection

### Research Question
Which robot platform should the course recommend for physical deployment?

### Options Evaluated

1. **Unitree Go2 Edu ($2,700)**
   - Type: Quadruped
   - SDK: Python, ROS 2 support via community packages
   - Pros: Most cost-effective mobile platform, good locomotion, educational discount available, active community
   - Cons: Quadruped (not humanoid), limited manipulation capabilities

2. **Unitree G1 Humanoid ($16,000)**
   - Type: Humanoid (bipedal + arms)
   - SDK: Python, ROS 2 integration
   - Pros: True humanoid form factor, manipulation capable, industry-grade
   - Cons: Very expensive, requires significant lab space and safety measures

3. **Robotis OP3 ($10,000)**
   - Type: Humanoid (small form factor)
   - SDK: ROS 2 native support, open-source
   - Pros: Educational design, manageable size, good documentation
   - Cons: Expensive, limited payload, slower adoption than Unitree

4. **Hiwonder Humanoids ($1,500-3,000)**
   - Type: Humanoid (various sizes)
   - SDK: Python, limited ROS support
   - Pros: Budget-friendly, good for kinematics demonstrations
   - Cons: Poor software support, unreliable hardware, not suitable for advanced AI work

5. **No Physical Robot (Simulation Only)**
   - Pros: Zero cost, accessible to all learners, safe
   - Cons: No sim-to-real validation, less motivating, misses tactile learning

### Decision: Unitree Go2 Edu (Primary) + G1 (Premium Option) + Simulation-Only Path

### Rationale

1. **Unitree Go2 Edu as Primary**:
   - Best cost-to-capability ratio for education labs
   - Mobile platform sufficient for navigation, perception, and VSLAM demonstrations
   - Community ROS 2 packages available (unitree_ros2)
   - Educational institutions can afford 1-2 units for shared lab access

2. **Unitree G1 as Premium/Aspirational**:
   - For institutions with larger budgets or research focus
   - Demonstrates full humanoid capabilities (bipedal + manipulation)
   - Content includes G1-specific examples as "advanced" material

3. **Simulation-Only as Default**:
   - **MOST IMPORTANT**: Course designed to be completable without physical robot
   - All concepts demonstrable in Gazebo, Unity, and Isaac Sim
   - Physical robot is enhancement, not requirement

### Implementation Strategy

- **Core Content**: Works with simulated humanoid (generic URDF model)
- **Go2 Extensions**: Optional chapters for quadruped-specific navigation
- **G1 Extensions**: Advanced capstone variant for full humanoid manipulation
- **Budget Alternative**: Mention Hiwonder only for basic kinematics visualization (not recommended for AI work)

### Tradeoffs Accepted

- **Go2 Not Humanoid**: Quadruped locomotion differs from bipedal; mitigation via simulation of humanoid models.
- **G1 Cost Prohibitive**: Most learners won't have access; mitigated by simulation-first approach.
- **Sim-to-Real Gap**: Without physical robot, learners miss real-world validation; addressed via detailed simulation domain randomization.

### References

- [Unitree Go2 Specs](https://www.unitree.com/go2)
- [Unitree G1 Announcement](https://www.unitree.com/g1)
- [unitree_ros2 GitHub](https://github.com/unitreerobotics/unitree_ros2)

---

## Decision 6: LLM Integration for Cognitive Planning

### Research Question
Which LLM should the course use for Vision-Language-Action (VLA) cognitive planning?

### Options Evaluated

1. **OpenAI GPT-4 Turbo**
   - API Cost: $0.01/1K input tokens, $0.03/1K output tokens
   - Pros: Best prompt engineering for robotics, structured outputs (JSON mode), function calling, high reliability
   - Cons: Requires API key, ongoing costs, internet dependency

2. **Anthropic Claude 3.5 Sonnet**
   - API Cost: $0.003/1K input tokens, $0.015/1K output tokens (cheaper than GPT-4)
   - Pros: Excellent reasoning, good structured outputs, cost-effective
   - Cons: Less community robotics examples, function calling less robust than GPT-4

3. **Open-Source (Meta Llama 3, Mistral)**
   - Cost: Free (self-hosted) or cheap inference (Replicate, Together.ai)
   - Pros: No API key required, can run locally (if sufficient VRAM)
   - Cons: Weaker reasoning for complex robotics tasks, requires more prompt engineering, inconsistent outputs

4. **GPT-3.5 Turbo (Legacy)**
   - Cost: $0.0015/1K input, $0.002/1K output (very cheap)
   - Pros: Very low cost, fast inference
   - Cons: Significantly weaker reasoning than GPT-4, less reliable for multi-step planning

### Decision: GPT-4 Turbo (Primary) + Claude 3.5 Sonnet (Alternative)

### Rationale

1. **GPT-4 Turbo for Primary Teaching**:
   - Best-in-class for translating natural language → structured ROS 2 actions
   - Function calling enables clean integration with ROS 2 action servers
   - JSON mode ensures parseable outputs
   - Extensive community examples for robotics (prompt patterns, error handling)

2. **Claude 3.5 Sonnet as Cost-Conscious Alternative**:
   - 70% cheaper than GPT-4 for similar quality
   - Good reasoning for most VLA tasks
   - Course includes Claude-specific prompt variations

3. **Open-Source as Advanced Exercise**:
   - Appendix chapter on "Running Local LLMs" for learners interested in self-hosting
   - Not recommended for primary VLA module due to complexity and inconsistency

### Implementation Details

**API Setup**:
- Both OpenAI and Anthropic require API keys (free tier available for initial testing)
- Cost estimate for Module 4: $5-15 per learner (assumes 500K tokens for VLA experimentation)

**Prompt Engineering Patterns**:
- Few-shot examples for ROS 2 action generation
- Chain-of-thought prompting for multi-step tasks
- Error handling and retry strategies

**Structured Output Format**:
```json
{
  "task": "navigate_and_grasp",
  "steps": [
    {"action": "navigate", "params": {"goal_x": 2.0, "goal_y": 3.0}},
    {"action": "detect_object", "params": {"class": "cup"}},
    {"action": "grasp", "params": {"object_id": "cup_01"}}
  ]
}
```

### Tradeoffs Accepted

- **API Costs**: Learners must budget $10-20 for Module 4; mitigated by free tier and cost estimation tools.
- **Internet Dependency**: VLA pipeline requires API access; mitigated by caching responses and offline replay for testing.

### References

- [OpenAI GPT-4 Pricing](https://openai.com/pricing)
- [Anthropic Claude Pricing](https://www.anthropic.com/pricing)
- [LangChain Robotics Examples](https://python.langchain.com/docs/use_cases/robotics)

---

## Decision 7: Hosting Platform

### Research Question
Where should the Docusaurus book be hosted for learner access?

### Options Evaluated

1. **GitHub Pages**
   - Cost: Free for public repos
   - Build: GitHub Actions automatic deployment
   - Custom Domain: Supported (CNAME)
   - Pros: Free, integrated with Git, simple setup, automatic SSL
   - Cons: No server-side rendering, basic analytics, GitHub outages affect site

2. **Vercel**
   - Cost: Free for hobby projects (generous limits)
   - Build: Automatic on push, optimized for Next.js/React
   - Pros: Excellent performance, preview deployments, serverless functions, analytics
   - Cons: Advanced features require paid plan, vendor lock-in

3. **Netlify**
   - Cost: Free for personal projects
   - Build: Automatic on push, split testing, forms
   - Pros: Feature-rich, good performance, form handling, authentication
   - Cons: Build minute limits on free tier, advanced features paid

4. **Self-Hosted (AWS S3 + CloudFront)**
   - Cost: ~$1-5/month depending on traffic
   - Pros: Full control, scalable, professional setup
   - Cons: Requires AWS knowledge, manual SSL setup (ACM), not free

### Decision: GitHub Pages (Primary Recommendation)

### Rationale

1. **Zero Cost**: Free for public repositories, ideal for open educational content.
2. **Simplicity**: Minimal configuration, automatic deployment via GitHub Actions.
3. **Git Integration**: Source control and hosting in one place, easy for learners to fork/contribute.
4. **Sufficient Performance**: Static site delivery fast enough for documentation; no server-side needs.
5. **Custom Domain Support**: Can use custom domain (e.g., physicalai.edu) if desired.
6. **Automatic SSL**: HTTPS enabled by default via Let's Encrypt.

### Alternative: Vercel (For Advanced Users)

Recommend Vercel if:
- Learners want preview deployments for content PRs
- Need server-side rendering for dynamic components (e.g., live code playgrounds)
- Want analytics and A/B testing for content effectiveness

### Implementation Details

**GitHub Pages Workflow**:
```yaml
# .github/workflows/deploy.yml
name: Deploy Docusaurus to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

**Custom Domain Setup** (Optional):
1. Add CNAME file to `static/` directory
2. Configure DNS A records to GitHub Pages IPs
3. Enable HTTPS in repository settings

### Tradeoffs Accepted

- **No Server-Side Features**: Cannot run server-side code or databases; acceptable for static documentation.
- **GitHub Dependency**: Site availability tied to GitHub uptime (historically 99.9%+).

### References

- [Docusaurus Deployment Guide](https://docusaurus.io/docs/deployment)
- [GitHub Pages Documentation](https://docs.github.com/en/pages)

---

## Decision 8: Content Accessibility Standards

### Research Question
What accessibility standards should the course meet?

### Decision: WCAG 2.1 Level AA Compliance

### Rationale

1. **Legal Compliance**: Many educational institutions require WCAG 2.1 AA for accessibility.
2. **Inclusive Learning**: Ensures learners with disabilities can access content (screen readers, keyboard navigation, color contrast).
3. **Docusaurus Support**: Docusaurus has built-in accessibility features that support WCAG compliance.
4. **Best Practice**: Industry standard for web accessibility.

### Implementation Requirements

**Perceivable**:
- ✅ Alt text for all images and diagrams
- ✅ Captions for video demonstrations
- ✅ Color contrast ratio ≥4.5:1 for text, ≥3:1 for large text
- ✅ Text alternatives for non-text content

**Operable**:
- ✅ Full keyboard navigation (no mouse-only interactions)
- ✅ Skip navigation links for screen readers
- ✅ Focus indicators visible on all interactive elements
- ✅ No time-based content (or provide controls)

**Understandable**:
- ✅ Clear heading hierarchy (H1 → H2 → H3 logical structure)
- ✅ Consistent navigation across pages
- ✅ Error messages in plain language
- ✅ Reading level appropriate for target audience (Flesch-Kincaid 10-12)

**Robust**:
- ✅ Valid HTML5 markup
- ✅ ARIA labels for custom components
- ✅ Compatible with assistive technologies (NVDA, JAWS, VoiceOver)

### Testing Strategy

1. **Automated**: Lighthouse accessibility audits (target score >90)
2. **Manual**: Screen reader testing (NVDA on Windows, VoiceOver on macOS)
3. **Keyboard**: Full site navigation using Tab, Enter, Escape, Arrow keys
4. **Color Contrast**: WebAIM Contrast Checker for all text/background combinations

### References

- [WCAG 2.1 Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)
- [Docusaurus Accessibility](https://docusaurus.io/docs/accessibility)

---

## Decision 9: Code Example Documentation Standards

### Research Question
What standards should code examples follow for clarity and reproducibility?

### Decision: Comprehensive Code Example Standard

### Format Requirements

Every code example must include:

1. **Filename & Language**
   ```python
   # filename: hello_ros2_publisher.py
   ```

2. **Description**
   - What the code does (1-2 sentences)
   - Prerequisites (ROS 2 packages, dependencies)

3. **Dependencies**
   ```bash
   # Required ROS 2 packages
   sudo apt install ros-humble-rclpy ros-humble-std-msgs
   ```

4. **Complete, Runnable Code**
   - No pseudocode or omissions
   - No "..." placeholders
   - Syntax-highlighted with Prism/Shiki

5. **Expected Output**
   ```
   [INFO] [publisher]: Publishing: "Hello ROS 2: 0"
   [INFO] [publisher]: Publishing: "Hello ROS 2: 1"
   ```

6. **How to Run**
   ```bash
   # Terminal 1: Source ROS 2 and run publisher
   source /opt/ros/humble/setup.bash
   python3 hello_ros2_publisher.py

   # Terminal 2: Echo the topic
   ros2 topic echo /hello_topic
   ```

7. **Troubleshooting**
   - Common errors and solutions
   - How to verify success

### Copy-to-Clipboard Feature

All code blocks include copy button (Docusaurus built-in):
```jsx
<CodeBlock language="python" showLineNumbers>
{codeString}
</CodeBlock>
```

### Downloadable Archives

All major code examples available as ZIP downloads:
- `static/code/ros2-packages/hello_ros2.zip`
- Includes: source code, README, dependencies.txt, run_example.sh

### Version Pinning

All dependencies specify versions:
```bash
# requirements.txt
rclpy==3.3.11  # ROS 2 Humble
numpy==1.24.3
opencv-python==4.8.1
```

### Rationale

- **Reproducibility**: Learners can copy-paste and run without modification.
- **Troubleshooting**: Reduces support burden by preempting common issues.
- **Offline Access**: Downloadable archives enable offline learning.
- **Version Stability**: Pinned versions prevent breaking changes from dependency updates.

### References

- [Docusaurus Code Blocks](https://docusaurus.io/docs/markdown-features/code-blocks)

---

## Best Practices Summary

### ROS 2 Package Structure

Follow standard ROS 2 package conventions:
```
my_robot_package/
├── package.xml          # Package manifest
├── setup.py             # Python package setup
├── setup.cfg            # Setup configuration
├── my_robot_package/    # Python module
│   ├── __init__.py
│   ├── my_node.py       # Node implementation
│   └── utils.py         # Helper functions
├── launch/              # Launch files
│   └── my_robot_launch.py
├── config/              # Configuration files
│   └── params.yaml
├── urdf/                # Robot models
│   └── my_robot.urdf
└── README.md            # Package documentation
```

### Docusaurus Content Organization

- **One Concept Per Page**: Each lesson = one Markdown file
- **Clear Hierarchy**: Module → Chapter → Lesson
- **Consistent Frontmatter**:
  ```markdown
  ---
  id: nodes-topics
  title: ROS 2 Nodes and Topics
  sidebar_label: Nodes & Topics
  sidebar_position: 2
  ---
  ```
- **Internal Links**: Use relative paths (`[Next Lesson](../ch2-python-rclpy/first-node.md)`)

### Educational Content Patterns

- **Prerequisites Section**: List required prior knowledge and dependencies
- **Learning Objectives**: Bullet list of "By the end of this lesson, you will be able to..."
- **Hands-On Exercise**: At least one practical exercise per lesson
- **Check Your Understanding**: Quiz or reflection questions
- **What's Next**: Link to next lesson and suggested practice

### Diagram Standards

- **Mermaid.js for Architecture**: Embedded flowcharts, sequence diagrams, class diagrams
- **ASCII Art for Simple Diagrams**: Terminal-friendly for SSH learners
- **PNG for Complex Visuals**: Export from draw.io or similar, include source file in repo
- **Alt Text Mandatory**: Describe diagram content for screen readers

---

## Technology Stack Summary

| Category | Choice | Version/Spec | Rationale |
|----------|--------|--------------|-----------|
| **ROS 2** | Humble Hawksbill | LTS (2022-2027) | Stability, Isaac ROS compatibility |
| **Ubuntu** | 22.04 LTS (Jammy) | LTS (2022-2027) | Aligns with ROS 2 Humble |
| **Python** | 3.10+ | Standard with Ubuntu 22.04 | ROS 2 rclpy compatibility |
| **Simulation** | Gazebo Fortress | Gazebo 7.x | Modern ROS 2 integration |
| **Visualization** | Unity 2021.3 LTS | Optional | High-fidelity graphics |
| **AI Platform** | NVIDIA Isaac Sim | 2023.1+ | Photorealistic sim, perception |
| **Edge Compute** | Jetson Orin Nano | 8GB variant ($499) | Cost-effective AI edge |
| **Robot** | Unitree Go2 Edu | Quadruped ($2,700) | Affordable mobile platform |
| **LLM** | GPT-4 Turbo | API (OpenAI) | Best cognitive planning |
| **Documentation** | Docusaurus | 3.x | React-based static site |
| **Hosting** | GitHub Pages | Free | Zero cost, Git integration |
| **Search** | Algolia DocSearch | Free for open source | Fast, accurate search |

---

## Open Questions & Future Research

### Resolved

All major technology decisions finalized.

### Remaining (For Phase 1)

1. **Specific URDF Model**: Choose or create humanoid URDF for simulation examples
   - Options: Adapt Unitree G1 URDF, use generic humanoid (e.g., RoboticsLab's humanoid), or create custom simplified model
   - Decision Needed: Module 1 content development

2. **Isaac Sim Docker vs. Native**: Finalize recommendation for cloud deployment
   - Docker: Easier setup, portable
   - Native: Better performance, simpler troubleshooting
   - Decision Needed: Module 3 research phase

3. **Video Hosting**: Where to host video demonstrations
   - YouTube: Free, embeddable, large audience
   - Vimeo: Professional, no ads, privacy controls
   - Self-hosted: Full control, bandwidth costs
   - Decision Needed: Content production phase

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2025-12-05 | 1.0.0 | Initial research document with all major technology decisions |

---

**Status**: Complete - Ready for Phase 1 (Data Model & Contracts)
**Next Steps**: Create `data-model.md`, `contracts/`, and `quickstart.md`
