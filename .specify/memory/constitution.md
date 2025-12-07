<!--
    Sync Impact Report
    - Version change: 0.0.0 → 1.0.0
    - List of modified principles:
        - None → Principle 1: Library-First
        - None → Principle 2: CLI Interface
        - None → Principle 3: Test-First (NON-NEGOTIABLE)
        - None → Principle 4: Integration Testing
        - None → Principle 5: Runnable Examples
        - None → Principle 6: Simplicity
    - Added sections:
        - Core Principles
        - Additional Constraints
        - Development Workflow
        - Governance
    - Removed sections:
        - None
    - Templates requiring updates:
        - ✅ .specify/templates/plan-template.md
        - ✅ .specify/templates/spec-template.md
        - ✅ .specify/templates/tasks-template.md
        - ✅ .gemini/commands/sp.constitution.toml
        - ✅ my-website/README.md
    - Follow-up TODOs:
        - None
-->
# Book on Physical AI & Humanoid Robotics Constitution

## Core Principles

### Principle 1: Library-First
Every feature starts as a standalone library; Libraries must be self-contained, independently testable, and documented. A clear purpose is required for each library; no organizational-only libraries.

### Principle 2: CLI Interface
Every library exposes its functionality via a Command Line Interface (CLI). The protocol for text I/O is stdin/args → stdout, with errors directed to stderr. Both JSON and human-readable formats should be supported.

### Principle 3: Test-First (NON-NEGOTIABLE)
Test-Driven Development (TDD) is mandatory. Tests must be written and approved by the user, fail before implementation, and the Red-Green-Refactor cycle must be strictly enforced.

### Principle 4: Integration Testing
Integration tests are required for new library contract tests, contract changes, inter-service communication, and shared schemas.

### Principle 5: Runnable Examples
Include runnable examples for key technologies, such as ROS 2, URDF, Gazebo/Unity, Isaac, and Nav2.

### Principle 6: Simplicity
Start simple and adhere to the "You Ain't Gonna Need It" (YAGNI) principle.

## Additional Constraints
Technology stack requirements, compliance standards, and deployment policies will be defined in this section.

## Development Workflow
Code review requirements, testing gates, and the deployment approval process will be defined in this section.

## Governance
This Constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs and reviews must verify compliance with this Constitution. Complexity must be justified.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07