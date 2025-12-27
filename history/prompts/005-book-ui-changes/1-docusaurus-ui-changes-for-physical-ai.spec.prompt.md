---
id: 1
title: Docusaurus UI changes for Physical AI & Humanoid Robotics Book
stage: spec
date_iso: 2025-12-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: 005-book-ui-changes
branch: 005-book-ui-changes
user: Claude
command: /sp.specify inside `my-frontend` folder. Requirement: UI/text changes only, fully responsive, no breaking changes.
labels:
  - ui-changes
  - docusaurus
  - book-landing-page
  - physical-ai
  - humanoid-robotics
links:
  spec: specs/005-book-ui-changes/spec.md
  ticket: null
  adr: null
  pr: null
files_yaml:
  - specs/005-book-ui-changes/spec.md
  - specs/005-book-ui-changes/checklists/requirements.md
tests_yaml:
  - spec validation
  - requirements checklist
prompt_text: " inside `my-frontend` folder. \nRequirement: UI/text changes only, fully responsive, no breaking changes.\n\n1) Navbar\n- Replace navbar title "AI/Spec-Driven Book" with "Home".\n- On homepage, replace text "AI/Spec-Driven Book" with \n  Embedded RAG Chatbot" with: "Physical AI & Humanoid Robotics Book". \n\n2) Homepage Button\n- Replace "Docusaurus Tutorial – 5min" button with "Read".\n- On click, navigate to Docs page.\n\n3) Homepage Content\n- Remove all default Docusaurus tutorial/promotional content.\n- Add clean, minimal, book-related content (Physical AI Humanoid Robotics). \n- Homepage should look like a technical book landing page.\n\n4) Docs Chapters Numbering\n- Keep all 4 modules and their content unchanged.\n- In Module 1 and Module 4, add chapter numbers before titles:\n  Chapter 1, Chapter 2, Chapter 3.\n- Do not change docs content or structure.\n\nRules:\n- Do not change docs text.\n- Do not break routing.\n- Ensure mobile + desktop responsiveness.\n- Keep project build & deploy ready.\n\nApply all changes only in `my-frontend`."
response_text: "Created specification for Docusaurus UI changes to transform the site into a Physical AI & Humanoid Robotics Book landing page. The spec includes user stories for navigation, branding, and content organization, with functional requirements and success criteria that are measurable and technology-agnostic."
outcome: Specification successfully created with user stories, functional requirements, and success criteria. The feature branch 005-book-ui-changes was created, and the spec file was written following the template structure. Quality validation checklist was completed with all items passing.
evaluation: The specification meets all requirements with clear user scenarios, testable requirements, and measurable success criteria. Ready for planning phase.
---

# PHR: Docusaurus UI changes for Physical AI & Humanoid Robotics Book

## Summary
Created specification for Docusaurus UI changes to transform the site into a Physical AI & Humanoid Robotics Book landing page. The spec includes user stories for navigation, branding, and content organization, with functional requirements and success criteria that are measurable and technology-agnostic.

## Details
- **Date**: 2025-12-28
- **Stage**: spec
- **Surface**: agent
- **Model**: claude-sonnet-4-5-20250929
- **Feature**: 005-book-ui-changes
- **Branch**: 005-book-ui-changes

## Files Created/Modified
- specs/005-book-ui-changes/spec.md
- specs/005-book-ui-changes/checklists/requirements.md

## Outcome
Specification successfully created with user stories, functional requirements, and success criteria. The feature branch 005-book-ui-changes was created, and the spec file was written following the template structure. Quality validation checklist was completed with all items passing.

## Evaluation
The specification meets all requirements with clear user scenarios, testable requirements, and measurable success criteria. Ready for planning phase.