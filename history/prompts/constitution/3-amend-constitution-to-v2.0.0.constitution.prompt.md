---
id: 3
title: Amend constitution to v2.0.0
stage: constitution
date: 2025-12-07
surface: agent
model: gemini
feature: 001-ros2-learning-module
branch: 001-ros2-learning-module
user: hanzalarehman
command: /sp.constitution This project specifies the creation of a book on Physical AI and Humanoid Robotics built with Docusaurus, deployed on GitHub Pages, and integrated with a RAG chatbot using OpenAI Agents/ChatKit, FastAPI, Neon Postgres, and Qdrant, with all content produced through Spec-Kit Plus and Claude Code. The work must prioritize technical accuracy across robotics, AI, and simulation; specification-driven development; clarity for upper-division CS and robotics students; reproducible code and deployment steps; and safe, correct descriptions of robot control. All claims must be verifiable from reputable sources with at least 40% academically credible references, consistent APA or IEEE citations, zero plagiarism in text and code, Flesch-Kincaid readability of grade 10–14, and coding practices aligned with ROS 2, FastAPI, and TypeScript standards; RAG retrieval must map cleanly to book sections. The book should contain 10–14 chapters (35k–50k words) with runnable examples for ROS 2, URDF, Gazebo/Unity, Isaac, Nav2, Whisper VLA pipelines, and the RAG backend, along with diagrams illustrating system architecture and data flow; the chatbot must answer only from the book or user-selected text. Success is defined by successful GitHub Pages deployment, a fully functional integrated RAG chatbot, accurate and well-sourced content, zero plagiarism, and reproducible simulation and humanoid robotics examples.
labels: [constitution, documentation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
- .specify/templates/plan-template.md
- .specify/templates/tasks-template.md
- .gemini/commands/sp.constitution.toml
tests:
- none
---

## Prompt

/sp.constitution This project specifies the creation of a book on Physical AI and Humanoid Robotics built with Docusaurus, deployed on GitHub Pages, and integrated with a RAG chatbot using OpenAI Agents/ChatKit, FastAPI, Neon Postgres, and Qdrant, with all content produced through Spec-Kit Plus and Claude Code. The work must prioritize technical accuracy across robotics, AI, and simulation; specification-driven development; clarity for upper-division CS and robotics students; reproducible code and deployment steps; and safe, correct descriptions of robot control. All claims must be verifiable from reputable sources with at least 40% academically credible references, consistent APA or IEEE citations, zero plagiarism in text and code, Flesch-Kincaid readability of grade 10–14, and coding practices aligned with ROS 2, FastAPI, and TypeScript standards; RAG retrieval must map cleanly to book sections. The book should contain 10–14 chapters (35k–50k words) with runnable examples for ROS 2, URDF, Gazebo/Unity, Isaac, Nav2, Whisper VLA pipelines, and the RAG backend, along with diagrams illustrating system architecture and data flow; the chatbot must answer only from the book or user-selected text. Success is defined by successful GitHub Pages deployment, a fully functional integrated RAG chatbot, accurate and well-sourced content, zero plagiarism, and reproducible simulation and humanoid robotics examples.

## Response snapshot

Updated the project constitution to version 2.0.0, incorporating new principles based on the user's input. Also updated the plan, tasks, and constitution command templates to align with the new constitution.

## Outcome

- ✅ Impact: Major update to the project's guiding principles.
- 🧪 Tests: Not applicable.
- 📁 Files: 4 files updated.
- 🔁 Next prompts: none
- 🧠 Reflection: The process of updating the constitution and propagating the changes to the templates was successful.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
