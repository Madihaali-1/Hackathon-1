<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.0.0 (initial constitution)
- Added sections: Core Principles (6), Additional Constraints, Development Workflow, Governance
- Templates requiring updates: N/A (initial creation)
- Follow-up TODOs: None
-->
# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First Workflow
All development begins with comprehensive specifications using Spec-Kit Plus. Every feature, API endpoint, and system component must have detailed requirements documented before implementation. This ensures clear understanding, testable acceptance criteria, and reproducible development.

### II. Technical Accuracy and Source Integrity
All content and code must be grounded in official, authoritative sources. No hallucinated responses or fabricated information is acceptable. All technical claims must be verifiable from official documentation or established best practices.

### III. Developer-Focused Writing
All documentation, code comments, and explanations must prioritize clarity for developers. Technical content should be practical, actionable, and include runnable examples with clear setup instructions.

### IV. Reproducible Setup and Deployment
Every component of the system must be deployable with clear, step-by-step instructions. All environments must be reproducible from configuration files. GitHub Actions or similar CI/CD workflows should automate deployment processes.

### V. Content-First RAG Architecture
The RAG chatbot must be grounded only in book content or user-selected text. No external knowledge sources should be used during retrieval-augmented generation. This ensures accuracy and prevents hallucinations.

### VI. End-to-End System Integration
All components (Docusaurus book, RAG chatbot, backend services, vector databases) must work together seamlessly. Integration points must be clearly defined and tested as a complete system.

## Additional Constraints

- GitHub-based source control with clear branching strategy and pull request workflows
- Technology stack: Docusaurus for book, OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud
- No hallucinated responses from the RAG system - strict grounding in provided content required
- All code must be well-documented with clear README files and usage examples
- Deployment to GitHub Pages for the book with integrated chatbot functionality

## Development Workflow

- All features must be specified using Spec-Kit Plus templates (spec.md, plan.md, tasks.md)
- Code reviews must verify compliance with all constitution principles
- All changes must pass automated tests before merging
- Documentation updates must accompany all feature implementations
- Regular compliance reviews to ensure adherence to principles

## Governance

This constitution supersedes all other development practices and guidelines. All team members must understand and follow these principles. Amendments require explicit documentation of changes, approval from project maintainers, and migration plans for existing code. All pull requests and code reviews must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-25 | **Last Amended**: 2025-12-25
