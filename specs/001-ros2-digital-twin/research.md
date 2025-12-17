# Research Notes: Digital Twin Simulation Module

## Phase 0 Research Findings

This `research.md` documents the initial technical decisions and their rationale, based on the feature specification (`spec.md`) and the established `plan.md`. There were no explicit "NEEDS CLARIFICATION" tags in the initial technical context, indicating a clear direction based on the project's requirements.

### Language/Version Decisions

- **Decision**: Utilize Python 3.11 for ROS 2 nodes, C# for Unity scripting, and JavaScript/TypeScript for Docusaurus content.
- **Rationale**: Python with `rclpy` is widely used and provides a suitable learning curve for students with basic ROS 2 knowledge. C# is the native and primary language for Unity development. JavaScript/TypeScript is the foundational technology for Docusaurus.
- **Alternatives Considered**: C++ for ROS 2 nodes (offering higher performance but increased complexity, which might be less suitable for a learning module focused on concepts rather than raw performance optimization); alternative web frameworks (rejected as Docusaurus is the established platform).

### Primary Dependencies Decisions

- **Decision**: Integrate and leverage existing ROS 2, Gazebo, Unity, and Docusaurus platforms.
- **Rationale**: These technologies are explicitly mentioned in the feature specification and form the core components of the digital twin simulation module.
- **Alternatives Considered**: Exploring other simulation platforms (e.g., Webots, MuJoCo) or documentation frameworks (e.g., Sphinx), or directly using DDS for ROS 2 communication (these were not considered as the specified tools are foundational to the course).

### Storage Decisions

- **Decision**: Content will be primarily stored as Markdown files within the Docusaurus documentation structure.
- **Rationale**: This aligns with the standard practices for Docusaurus-based documentation and is appropriate for static textual and code-based educational content.
- **Alternatives Considered**: Employing a database for content management (deemed unnecessary and overly complex for a Docusaurus documentation module).

### Testing Strategy Decisions

- **Decision**: Implement testing using native frameworks for each technology: `rostest` and `GTest` for ROS 2 components, `Unity Test Framework` for Unity, and `Jest` along with `Playwright` for Docusaurus.
- **Rationale**: Adhering to the idiomatic testing practices of each environment ensures robust validation and familiarity for students.
- **Alternatives Considered**: Developing custom, unified testing solutions (would introduce unnecessary complexity and deviation from standard practices).

### Target Platform Decisions

- **Decision**: The Docusaurus content will target web browsers. ROS 2 components will target Linux environments (specifically Ubuntu 22.04 for ROS 2 Humble). Unity simulations will be developed cross-platform, allowing for flexibility in student environments.
- **Rationale**: This aligns with the typical deployment and development environments for each technology, providing a realistic and accessible learning experience.
- **Alternatives Considered**: Restricting platform choices (would limit accessibility and educational scope).

### Project Type Classification

- **Decision**: Classify the overall project as a "Web" project (due to Docusaurus) that encapsulates "Robotics Simulation Content."
- **Rationale**: The primary delivery and interaction medium for the students will be the Docusaurus website, which hosts the educational material and integrates with the simulation components.
- **Alternatives Considered**: Classifying as a pure robotics project (would overlook the critical Docusaurus integration and content delivery aspect).

### Performance Goal Decisions

- **Decision**: Aim for real-time simulation performance in both Gazebo and Unity environments, and ensure fast page loading times for the Docusaurus website.
- **Rationale**: Real-time simulation is crucial for interactive learning and realistic robot behavior. Fast page loads contribute to a smooth and engaging user experience for the Docusaurus documentation.
- **Alternatives Considered**: Accepting lower performance targets (would negatively impact the quality of the learning experience).

### Constraints Adherence

- **Decision**: Strictly adhere to the existing Docusaurus structure and the established ROS 2 course curriculum.
- **Rationale**: This ensures seamless integration of the new module into the broader educational material, maintaining consistency and avoiding fragmentation.
- **Alternatives Considered**: Introducing significant deviations from existing structures (would create inconsistencies and potential confusion for students).

### Scale/Scope Definition

- **Decision**: The scope of this project is limited to delivering one new Docusaurus module, comprising three distinct chapters.
- **Rationale**: This scope is clearly defined by the project requirements, allowing for focused and efficient development.
- **Alternatives Considered**: Expanding the scope beyond the specified module (would lead to scope creep and potential delays).
