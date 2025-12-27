# Implementation Tasks: Docusaurus UI Changes for Physical AI & Humanoid Robotics Book

**Feature**: 005-book-ui-changes
**Created**: 2025-12-28
**Status**: Draft
**Plan**: specs/005-book-ui-changes/plan.md

## Phase 1: Setup Tasks

- [X] T001 Verify current project structure and confirm my-frontend directory exists
- [X] T002 [P] Check that docusaurus.config.js file exists in my-frontend directory
- [X] T003 [P] Check that src/pages/index.js file exists in my-frontend directory
- [X] T004 [P] Check that sidebars.js file exists in my-frontend directory
- [X] T005 [P] Run current build to verify project works before making changes
- [X] T006 Create backup of current configuration files

## Phase 2: Foundational Tasks

- [X] T007 [P] Update navbar title in docusaurus.config.js from "AI/Spec-Driven Book" to "Home"
- [X] T008 [P] Verify the correct path for documentation pages to link from homepage button
- [X] T009 [P] Identify all homepage content that needs to be replaced with book-specific content

## Phase 3: User Story 1 - Navigate to Book Content (Priority: P1)

**Goal**: As a visitor to the Physical AI & Humanoid Robotics Book website, I want to easily access the book content from the homepage so that I can start reading immediately.

**Independent Test Criteria**: Can be fully tested by visiting the homepage and clicking the "Read" button, which should navigate to the docs page where the book content is available.

- [X] T010 [US1] Replace "Docusaurus Tutorial – 5min" button with "Read" button on homepage
- [X] T011 [US1] Configure "Read" button to navigate to the documentation page
- [X] T012 [US1] Test that clicking "Read" button navigates to correct documentation page
- [X] T013 [US1] Verify navbar "Home" link still navigates to homepage correctly

## Phase 4: User Story 2 - View Book-Specific Branding (Priority: P1)

**Goal**: As a visitor to the website, I want to see clear branding that indicates this is about "Physical AI & Humanoid Robotics Book" rather than generic AI content, so that I understand the specific focus of the content.

**Independent Test Criteria**: Can be fully tested by visiting the homepage and verifying that the title and content reflect "Physical AI & Humanoid Robotics Book" rather than the old generic branding.

- [X] T014 [US2] Replace main title from "AI/Spec-Driven Book with Embedded RAG Chatbot" to "Physical AI & Humanoid Robotics Book"
- [X] T015 [US2] Remove all default Docusaurus tutorial/promotional content from homepage
- [X] T016 [US2] Add clean, minimal, book-related content focused on Physical AI & Humanoid Robotics
- [X] T017 [US2] Ensure homepage looks like a technical book landing page
- [X] T018 [US2] Verify navbar shows "Home" instead of "AI/Spec-Driven Book"

## Phase 5: User Story 3 - Access Structured Book Content (Priority: P2)

**Goal**: As a reader of the book, I want to see clearly numbered chapters in the documentation so that I can follow the content in a logical, sequential order.

**Independent Test Criteria**: Can be fully tested by navigating to Module 1 and Module 4 documentation pages and verifying that chapters are numbered as "Chapter 1", "Chapter 2", "Chapter 3".

- [X] T019 [US3] Add "Chapter 1", "Chapter 2", "Chapter 3" prefixes to Module 1 documentation items in sidebars.js
- [X] T020 [US3] Add "Chapter 1", "Chapter 2", "Chapter 3" prefixes to Module 4 documentation items in sidebars.js
- [X] T021 [US3] Verify existing documentation content remains unchanged
- [X] T022 [US3] Test that chapter numbering appears correctly in documentation sidebar

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T023 Verify all changes maintain responsive design on mobile and desktop
- [X] T024 [P] Run build process to ensure no errors after all changes
- [X] T025 [P] Test navigation functionality across all pages
- [X] T026 [P] Verify all existing routing continues to function without breaking links
- [X] T027 [P] Perform final visual check across different browsers
- [X] T028 [P] Run local server to verify all functionality works as expected

## Dependencies

User stories can be implemented in parallel after foundational tasks are completed:
- Phase 2 (Foundational) must be completed before Phases 3, 4, and 5
- Phase 3, 4, and 5 can be implemented in parallel
- Phase 6 must be completed after all other phases

## Parallel Execution Examples

**Parallel Tasks by File**:
- T002, T003, T004: Check different configuration files in parallel
- T010, T014: Update homepage content in parallel
- T019, T020: Update different modules in sidebars.js in parallel
- T023, T024, T025: Testing tasks can run in parallel after implementation

## Implementation Strategy

**MVP Scope**: Complete User Story 1 (T010-T013) to provide core functionality of navigating to book content.

**Incremental Delivery**:
1. MVP: Homepage navigation to documentation
2. Enhancement: Branding and content updates
3. Enhancement: Chapter numbering in documentation
4. Polish: Responsive design and testing verification

## Quality Assurance

- All UI changes must be responsive on mobile, tablet, and desktop
- No existing functionality should be broken
- Build process must continue to work without errors
- All existing routing must remain functional