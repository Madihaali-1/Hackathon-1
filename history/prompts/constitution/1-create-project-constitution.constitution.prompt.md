---
id: 1
title: create-project-constitution
stage: constitution
date: 2025-12-25
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: Claude Code User
command: /sp.constitution
labels: [constitution, setup, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
- .specify/templates/plan-template.md
- .specify/templates/tasks-template.md
- README.md
tests:
- none
---

## Prompt

Project: AI/Spec-Driven Book with Embedded RAG Chatbot

Core principles:
-Spec-first workflow using Spec-Kit Plus
-Technical accuracy from official sources
-Clear, developer-focused writing
-Reproducible setup and deployment

Key standards:
-Book written with Docusaurus and deployed on GitHub Pages
-RAG chatbot grounded only in book content or user-selected text
-Stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud
-Runnable, well-documented code

Constraints:
-GitHub-based source control
-No hallucinated responses
-End-to-end reproducibility

Success criteria:
-Live book on GitHub Pages
-Fully functional embedded RAG chatbot
-All specs implemented via Spec-Kit Plus

## Response snapshot

Created a comprehensive project constitution for the AI/Spec-Driven Book with Embedded RAG Chatbot project. The constitution includes six core principles:

1. Spec-First Workflow
2. Technical Accuracy and Source Integrity
3. Developer-Focused Writing
4. Reproducible Setup and Deployment
5. Content-First RAG Architecture
6. End-to-End System Integration

Updated the plan template to include constitution compliance checks and the tasks template to emphasize constitution alignment. Updated the README to reflect the project's purpose and principles.

## Outcome

- ✅ Impact: Established foundational governance for the project with clear principles
- 🧪 Tests: No tests required for constitution creation
- 📁 Files: Updated constitution, plan template, tasks template, and README
- 🔁 Next prompts: Project can now proceed with feature specifications following the constitution
- 🧠 Reflection: Constitution provides clear guidance for future development decisions

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating feature specifications following the constitution