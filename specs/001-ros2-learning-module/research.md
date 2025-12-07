# Research: ROS 2 Learning Module

## Research Tasks

### 1. ROS 2 Installation on macOS and Windows

**Decision**: The module will primarily target Ubuntu 22.04 with ROS 2 Humble, as it is the most common and officially supported platform. However, to support students on other operating systems, the module will include links to the official ROS 2 installation guides for macOS and Windows. It will also include a disclaimer that these platforms are not the primary focus and may have some differences.

**Rationale**: Providing full, tested instructions for all three platforms is beyond the scope of this module and would significantly increase its length and complexity. Linking to the official documentation is the most effective way to help students on other platforms without compromising the quality of the main content.

**Alternatives considered**:
- Providing detailed installation instructions for all three platforms: Rejected due to complexity and scope creep.
- Only supporting Ubuntu: Rejected as it would exclude a significant number of students.

### 2. Assumed Python Proficiency for `rclpy` Examples

**Decision**: The module will assume a basic to intermediate level of Python proficiency. This means students should be comfortable with:
- Basic Python syntax (variables, data types, loops, conditionals, functions).
- Object-oriented programming (classes and objects).
- Basic command-line usage.

The module will not teach Python itself, but the code examples will be well-commented to explain the `rclpy`-specific parts.

**Rationale**: The target audience is upper-division CS and robotics students, who are expected to have a foundational knowledge of Python. Assuming this level of proficiency allows the module to focus on the core ROS 2 concepts without getting bogged down in basic programming instruction.

**Alternatives considered**:
- Assuming zero Python knowledge: Rejected as it would make the module too long and basic for the target audience.
- Assuming expert Python knowledge: Rejected as it would make the examples difficult to understand for some students.
