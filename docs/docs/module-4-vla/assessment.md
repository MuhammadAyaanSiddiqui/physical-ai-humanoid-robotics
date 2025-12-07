# Module 4 Assessment: Voice-Controlled Cognitive Robotics System

## Project Overview

**Goal**: Build a complete voice-controlled robotics system integrating Whisper, LLM planning, and ROS 2 execution.

**Deadline**: End of Week 13 (3 weeks for Module 4)

**Submission Format**:
- GitHub repository with code
- Demo video (5-10 minutes)
- Technical report (PDF, 3-5 pages)

---

## Project Requirements

### Minimum Viable System (60 points)

1. **Voice Input** (15 points)
   - ✅ Capture audio from microphone (USB or ReSpeaker)
   - ✅ Transcribe using Whisper (cloud or local)
   - ✅ Handle at least 10 different voice commands
   - ✅ Display transcripts in real-time

2. **Language Understanding** (15 points)
   - ✅ Parse commands with spaCy or LLM
   - ✅ Extract intent and entities correctly
   - ✅ Handle basic variations ("pick the cup" vs "grab the cup")

3. **Action Planning** (15 points)
   - ✅ Use LLM (GPT-4/Claude) to generate action plans
   - ✅ Output valid JSON action sequences
   - ✅ Validate plans before execution
   - ✅ Handle at least 3 action types (navigate, pick, place)

4. **Robot Execution** (15 points)
   - ✅ Execute actions in ROS 2 (simulation or hardware)
   - ✅ Provide feedback during execution
   - ✅ Complete at least 3 compound tasks successfully
   - ✅ Log all actions and results

### Advanced Features (40 points)

5. **Visual Grounding** (10 points)
   - 🌟 Integrate camera feed
   - 🌟 Detect objects with YOLO or similar
   - 🌟 Map language descriptions to visual objects
   - 🌟 Handle spatial relations ("cup on the table")

6. **Error Handling** (10 points)
   - 🌟 Detect action failures
   - 🌟 Implement retry logic (3 attempts)
   - 🌟 Replan with LLM on persistent failures
   - 🌟 Ask clarifying questions for ambiguous commands

7. **Multi-Step Planning** (10 points)
   - 🌟 Handle compound commands ("go to kitchen and find cup")
   - 🌟 Execute sequential actions with dependencies
   - 🌟 Maintain context across multiple turns
   - 🌟 Optimize action sequences

8. **Human-Robot Interaction** (10 points)
   - 🌟 Provide natural language feedback
   - 🌟 Confirm understanding before executing
   - 🌟 Handle interruptions gracefully
   - 🌟 Support conversational follow-ups

### Bonus Points (20 points)

9. **Innovation** (up to 10 points)
   - Creative use of multi-modal sensors
   - Novel error recovery strategies
   - Real-world deployment on physical robot
   - Unique application domain

10. **Code Quality** (up to 10 points)
   - Clean, well-documented code
   - Comprehensive error handling
   - Unit tests for key components
   - Professional README and documentation

---

## Grading Rubric

| Category | Excellent (100%) | Good (75%) | Satisfactory (50%) | Needs Work (25%) |
|----------|------------------|------------|-------------------|------------------|
| **Voice Input** | Robust multi-mic setup, &lt;1s latency, 95%+ accuracy | USB mic, 1-2s latency, 85%+ accuracy | Basic mic, 2-3s latency, 70%+ accuracy | Frequent failures, poor quality |
| **Language Understanding** | Handles complex commands, context-aware | Parses most commands correctly | Basic parsing, misses some entities | Frequent parsing errors |
| **Action Planning** | Optimal plans, handles edge cases | Valid plans for common scenarios | Simple plans, some errors | Invalid or infeasible plans |
| **Robot Execution** | Smooth execution, real-time feedback | Reliable execution, basic feedback | Executes most actions | Frequent failures |
| **System Integration** | Seamless end-to-end flow | All components integrated | Partially integrated | Components don't work together |

---

## Test Scenarios

Your system must successfully complete at least **7 out of 10** test scenarios:

### Basic Tasks (Must Pass 5/5)

1. **Navigate**: "Go to the kitchen"
   - Expected: Robot navigates to kitchen location
   - Success: Reaches kitchen within 10 seconds

2. **Pick**: "Pick up the red cube"
   - Expected: Robot identifies and grasps red cube
   - Success: Cube held securely in gripper

3. **Place**: "Place the cube on the table"
   - Expected: Robot places cube at table location
   - Success: Cube resting on table surface

4. **Scan**: "Find the blue ball"
   - Expected: Robot scans environment and reports ball location
   - Success: Correct identification and pose estimate

5. **Compound**: "Go to the bedroom and turn on the light"
   - Expected: Navigate + manipulation sequence
   - Success: Both actions completed successfully

### Advanced Tasks (Must Pass 2/5)

6. **Ambiguous Reference**: "Pick it up" (after previous command)
   - Expected: Resolve "it" from context
   - Success: Correctly identifies object from history

7. **Spatial Reasoning**: "Pick up the cup on the left table"
   - Expected: Distinguish between multiple tables
   - Success: Selects correct table based on spatial location

8. **Error Recovery**: Block robot's path during navigation
   - Expected: Detect obstacle, replan route
   - Success: Reaches destination via alternative path

9. **Clarification**: "Pick up the cup" (when 3 cups visible)
   - Expected: Ask "Which cup?" and wait for response
   - Success: User clarifies, robot picks correct cup

10. **Multi-Turn**:
    - Turn 1: "Go to the kitchen"
    - Turn 2: "Pick up the red cup"
    - Turn 3: "Bring it to me"
    - Expected: Maintain context across 3 commands
    - Success: All actions completed as single workflow

---

## Submission Checklist

### Code Repository

- [ ] `README.md` with setup instructions
- [ ] `requirements.txt` or `package.json` with dependencies
- [ ] Source code in organized directory structure
- [ ] Launch files for ROS 2 nodes
- [ ] Configuration files (.env.example)
- [ ] Test scripts or notebooks

### Demo Video (5-10 minutes)

- [ ] Introduction (30s): Team, project goal
- [ ] System architecture overview (1min)
- [ ] Live demonstration of 3-5 test scenarios (3-5min)
- [ ] Error handling example (1min)
- [ ] Discussion of challenges and solutions (1-2min)
- [ ] Conclusion and future work (30s)

Upload to YouTube (unlisted) and include link in README.

### Technical Report (3-5 pages)

**Required Sections**:

1. **Introduction** (0.5 pages)
   - Problem statement
   - System objectives
   - Key contributions

2. **System Architecture** (1-1.5 pages)
   - Component diagram
   - Data flow diagram
   - Technology stack justification

3. **Implementation** (1.5-2 pages)
   - Voice input pipeline
   - LLM integration approach
   - ROS 2 action execution
   - Error handling strategies

4. **Evaluation** (0.5-1 page)
   - Test results for 10 scenarios
   - Performance metrics (latency, accuracy)
   - Limitations and failure cases

5. **Conclusion** (0.5 pages)
   - Summary of achievements
   - Lessons learned
   - Future improvements

**Format**: PDF, 11pt font, single-column, IEEE or ACM style

---

## Example Passing Submission

**Scenario**: Voice-controlled warehouse robot

**Features Implemented**:
- ✅ Whisper API transcription (800ms latency)
- ✅ GPT-4o-mini planning (1.5s latency)
- ✅ 12 supported commands
- ✅ ROS 2 Nav2 integration
- ✅ YOLOv8 object detection
- ✅ Retry logic (3 attempts)
- ✅ Clarification dialogues
- ✅ 8/10 test scenarios passing

**Demo Video Highlights**:
- Smooth voice → action flow
- Successful error recovery when object not found
- Clear visual feedback during execution

**Grade**: 85/100 (B+)
- Minimum viable: 60/60
- Advanced features: 20/40 (visual grounding + error handling)
- Bonus: 5/20 (code quality)

---

## Frequently Asked Questions

**Q: Can I use simulation instead of a physical robot?**
A: Yes! Isaac Sim, Gazebo, or even a simple 2D simulator is acceptable. Physical robots get bonus points.

**Q: Which LLM should I use?**
A: GPT-4o-mini is recommended (cheap, fast, good quality). Claude 3.5 Sonnet also works well. Open-source models (Llama 3) are fine but may require more prompt engineering.

**Q: Can I work in a team?**
A: Teams of 2-3 are allowed. Clearly indicate individual contributions in the report.

**Q: What if my system doesn't pass all 10 test scenarios?**
A: You need 7/10 to pass. Focus on getting the 5 basic scenarios working first, then attempt advanced ones.

**Q: How is the demo video graded?**
A: Quality of explanation (clear, concise) and demonstration (smooth, no excessive editing). Raw footage with voiceover is fine - doesn't need to be a Hollywood production!

---

## Resources

**Code Templates**:
- `/static/code/vla-examples/` - Complete reference implementations

**Tutorials**:
- Module 4 lessons (Chapters 11-14)
- [Whisper API Guide](https://platform.openai.com/docs/guides/speech-to-text)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)

**Support**:
- Course discussion forum
- Office hours: Wednesdays 2-4 PM
- TA email: robotics-ta@university.edu

---

## Submission

**Deadline**: End of Week 13 (check course calendar for exact date)

**Submit via**:
1. GitHub repository link (public or invite instructor)
2. Demo video link (YouTube unlisted)
3. Technical report PDF (upload to course portal)

**Late Policy**: -10% per day, up to 3 days. After that, 0 points.

---

**Good luck! 🤖🎉**

Remember: The goal is to integrate all Module 4 concepts into a working system. Focus on getting the core pipeline working first, then add advanced features if time permits.
