<!--
    Sync Impact Report
    - Version change: 1.0.0 → 2.0.0
    - List of modified principles:
        - Principle 1: Library-First → Technical Accuracy
        - Principle 2: CLI Interface → Specification-Driven Development
        - Principle 3: Test-First (NON-NEGOTIABLE) → Clarity and Accessibility
        - Principle 4: Integration Testing → Reproducibility and Verifiability
        - Principle 5: Runnable Examples → Safety and Correctness
        - Principle 6: Simplicity → Content Integrity
    - Added sections:
        - None
    - Removed sections:
        - None
    - Templates requiring updates:
        - ⏳ .specify/templates/plan-template.md
        - ⏳ .specify/templates/spec-template.md
        - ⏳ .specify/templates/tasks-template.md
        - ⏳ .gemini/commands/sp.constitution.toml
        - ⏳ my-website/README.md
    - Follow-up TODOs:
        - None
-->
# Constitution: Physical AI & Humanoid Robotics Book

## Core Principles

### Principle 1: Technical Accuracy
All content must be technically accurate across robotics, AI, and simulation. Claims must be verifiable from reputable sources, with a minimum of 40% academically credible references.

### Principle 2: Specification-Driven Development
All content, including text and code, must be produced through Spec-Kit Plus, adhering to a strict specification-driven development workflow.

### Principle 3: Clarity and Accessibility
Content must be written for clarity, targeting upper-division computer science and robotics students. The target Flesch-Kincaid readability grade is 10–14.

### Principle 4: Reproducibility and Verifiability
All code, examples, and deployment steps must be reproducible. This includes providing runnable examples for ROS 2, URDF, Gazebo/Unity, Isaac, Nav2, and the RAG backend.

### Principle 5: Safety and Correctness
Descriptions of robot control and all related code must be safe and correct.

### Principle 6: Content Integrity
There will be zero plagiarism in text and code. All sources must be cited using a consistent APA or IEEE format. The RAG chatbot must only use the book's content for its answers.

## Additional Constraints
- **Technology Stack**: Docusaurus, GitHub Pages, OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant.
- **Content Scope**: 10–14 chapters, totaling 35,000–50,000 words.
- **Coding Standards**: Adhere to ROS 2, FastAPI, and TypeScript best practices.

## Development Workflow
The development process will follow the guidelines and tools provided by Spec-Kit Plus. All work will be managed through specifications, plans, and tasks.

## Governance
This Constitution is the project's foundational document. Any amendments must be documented, approved, and include a migration plan for existing content and systems. All contributions and reviews must ensure compliance with these principles.

**Version**: 2.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
