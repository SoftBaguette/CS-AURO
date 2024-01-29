# Autonomous Mobile Robot Systems Assessment Plan

## 1. Task Definition
**Objective:** Develop a system where one to three TurtleBot3 Waffle Pi robots autonomously collect items in a simulated environment and return them to a home zone, considering obstacles and item respawning dynamics.

## 2. Deliverables
### 2.1 Implementation
- **Robot Operating System (ROS) Usage:** Utilize ROS 2 Humble Hawksbill and Gazebo Classic 11. Implement using Python (rclpy).
- **Robot Specifications:** Use TurtleBot3 Waffle Pi robots, with a maximum of three, all starting in the home zone.
- **Code Structure and Documentation:** Ensure modularity, clarity, and comprehensive documentation in a README file.
- **Criteria for Assessment:**
  - **Sophistication:** Intelligence and efficiency of the robotic system, particularly in a multi-robot scenario.
  - **System Architecture:** Modularity, component reusability, and interface design.
  - **ROS Concepts:** Breadth and correct usage of ROS features.
  - **Understandability:** Code structure, naming conventions, comments, and README quality.
- **Given Resources:** Base code for the simulated world and item detection; permissible to use 3rd party packages with citation.

### 2.2 Demonstration
- This section is not applicable for AI assistance.

### 2.3 Report (50%)
- **Report Structure:** Design, Implementation, Analysis, Evaluation, Safety and Ethics, Presentation.
- **Formatting:** Adhere to IEEE conference template, A4 size, IEEE referencing style.
- **Content Requirements:**
  - **Design:** System design description and justification with a high-level block diagram.
  - **Implementation:** Description and justification of the implementation, autonomy mechanism diagram.
  - **Analysis:** Description of experimental approach, presentation and interpretation of results.
  - **Evaluation:** Discussion of solution's strengths, weaknesses, and real-world transferability.
  - **Safety and Ethics:** Discussion on safety implications and ethical considerations.
- **Report Length:** No more than 4 pages (excluding references).

#### Submission Structure
- A ZIP file containing the report, README, rosgraph image, and the `src` directory with ROS packages.

## Plan of Action
1. **System Design & Architecture:**
   - Plan a scalable multi-robot system architecture.
   - Design efficient item retrieval and obstacle avoidance algorithms.
   - Determine the communication protocol among robots and with the home base.

2. **Implementation:**
   - Develop ROS nodes and packages.
   - Implement item detection, navigation, and coordination algorithms.
   - Test and refine in the Gazebo environment.

3. **Documentation:**
   - Create a comprehensive README file detailing setup, dependencies, and execution instructions.
   - Comment code for clarity and maintainability.

4. **Report Preparation:**
   - Design Section: Describe system architecture with a block diagram.
   - Implementation Section: Detail the development process and autonomy mechanism with a state machine diagram.
   - Analysis Section: Conduct experiments and present results quantitatively and qualitatively.
   - Evaluation Section: Critically assess the system's performance and real-world applicability.
   - Safety and Ethics Section: Discuss relevant safety and ethical considerations.
   - Ensure adherence to IEEE formatting and referencing standards.

5. **Testing & Validation:**
   - Conduct thorough testing in the simulated environment.
   - Validate system performance against task requirements.

6. **Final Review:**
   - Ensure all components are integrated and functioning as intended.
   - Review the report and submission package for completeness and compliance.

This plan aims to guide you through a structured approach to meet the assessment requirements efficiently. If you need specific guidance on any of these steps, feel free to ask.
