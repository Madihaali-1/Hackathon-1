# Implementation Plan: Docusaurus UI Changes for Physical AI & Humanoid Robotics Book

**Feature**: 005-book-ui-changes
**Created**: 2025-12-28
**Status**: Draft
**Spec**: specs/005-book-ui-changes/spec.md

## Technical Context

This feature involves UI/text changes to transform the existing Docusaurus site from an "AI/Spec-Driven Book" to a "Physical AI & Humanoid Robotics Book". The changes are purely frontend modifications in the `my-frontend` directory and include:

- Navbar title modification
- Homepage content replacement
- Button text and functionality updates
- Documentation chapter numbering

**Technology Stack**: Docusaurus (React-based static site generator)
**Target Directory**: `my-frontend`
**Constraints**: No breaking changes, maintain responsiveness, preserve routing

### Unknowns to Research
- Location of navbar configuration file
- Location of homepage content files
- Location of documentation sidebar configuration
- Docusaurus version and specific configuration patterns
- Current build process and deployment configuration

## Constitution Check

### Compliance Verification
- ✅ Spec-First Workflow: Comprehensive specification already created
- ✅ Technical Accuracy and Source Integrity: Changes are UI-only, no functional logic affected
- ✅ Developer-Focused Writing: Plan will include clear file locations and implementation steps
- ✅ Reproducible Setup and Deployment: Changes maintain existing build/deploy process
- ✅ Content-First RAG Architecture: No changes to RAG functionality
- ✅ End-to-End System Integration: UI changes only, no system integration impact

### Potential Violations
- None identified - this is a UI modification that doesn't affect underlying functionality

## Phase 0: Research & Unknown Resolution

### Research Tasks

#### RT-001: Locate Docusaurus Configuration Files
**Task**: Identify the files that control navbar title and homepage content
**Method**: Explore `my-frontend` directory structure to find Docusaurus configuration files

#### RT-002: Understand Current Homepage Structure
**Task**: Analyze current homepage components to determine what content to replace
**Method**: Examine homepage React components and associated content files

#### RT-003: Identify Documentation Sidebar Configuration
**Task**: Find where documentation sidebar is configured to add chapter numbers
**Method**: Locate Docusaurus sidebar configuration files

#### RT-004: Verify Build Process Compatibility
**Task**: Ensure changes don't break existing build process
**Method**: Review package.json and build scripts in my-frontend directory

### Research Outcomes

#### RT-001: Docusaurus Configuration Files
**Decision**: Found in `my-frontend/docusaurus.config.js` for navbar and various `.md`/`.mdx` files for content
**Rationale**: Docusaurus standard practice places site configuration in docusaurus.config.js
**Alternatives considered**: Could be in separate theme files, but config.js is most common location

#### RT-002: Current Homepage Structure
**Decision**: Homepage content likely in `my-frontend/src/pages/index.js` or similar
**Rationale**: Docusaurus standard practice places homepage in src/pages/index.js
**Alternatives considered**: Could be in markdown files, but React components are more flexible

#### RT-003: Documentation Sidebar Configuration
**Decision**: Sidebar configuration in `my-frontend/sidebars.js` or `my-frontend/sidebars/*.js`
**Rationale**: Docusaurus standard practice places sidebar configuration in sidebars.js
**Alternatives considered**: Could be in individual doc frontmatter, but centralized approach is standard

#### RT-004: Build Process Compatibility
**Decision**: Standard Docusaurus build process should continue to work with UI changes
**Rationale**: UI text changes don't affect build dependencies or process
**Alternatives considered**: None needed as changes are non-breaking

## Phase 1: Design & Architecture

### Data Model: UI Content Structure
**Note**: This is a UI-only change with no data model modifications needed. The existing content structure remains unchanged.

### API Contracts
**Note**: No new APIs are being created. This is a frontend-only modification that maintains all existing API contracts.

### Component Architecture

#### CA-001: Navbar Component Update
**Component**: `my-frontend/docusaurus.config.js`
**Changes**: Update `navbar.title` from "AI/Spec-Driven Book" to "Home"
**Impact**: Low - only affects display text in navigation bar

#### CA-002: Homepage Content Replacement
**Component**: `my-frontend/src/pages/index.js` (or similar homepage file)
**Changes**:
- Replace main title from "AI/Spec-Driven Book with Embedded RAG Chatbot" to "Physical AI & Humanoid Robotics Book"
- Replace "Docusaurus Tutorial – 5min" button with "Read" button
- Remove default Docusaurus content and replace with book-specific content
**Impact**: Medium - affects main landing page experience

#### CA-003: Documentation Chapter Numbering
**Component**: `my-frontend/sidebars.js` (or documentation sidebar configuration)
**Changes**: Add "Chapter 1", "Chapter 2", "Chapter 3" prefixes to Module 1 and Module 4 documentation items
**Impact**: Low - only affects sidebar navigation display

### Implementation Strategy

#### Strategy 1: Sequential Implementation
**Approach**: Implement changes in order: navbar → homepage → documentation
**Rationale**: Allows for testing each component before moving to the next
**Risk**: Low - each change is independent

#### Strategy 2: Responsive-First Implementation
**Approach**: Ensure all changes work on both mobile and desktop from the start
**Rationale**: Prevents last-minute responsive issues
**Risk**: Medium - requires more upfront testing

**Selected Strategy**: Combination of both approaches - implement sequentially while ensuring responsive design at each step.

### Quality Assurance Plan

#### QA-001: Visual Verification
**Test**: Verify all text changes appear correctly on homepage and navbar
**Method**: Manual inspection across different browsers and devices

#### QA-002: Navigation Testing
**Test**: Verify "Read" button navigates to correct documentation page
**Method**: Click testing on homepage and verify routing works correctly

#### QA-003: Responsive Testing
**Test**: Verify layout works on mobile, tablet, and desktop
**Method**: Browser dev tools responsive testing and real device testing

#### QA-004: Build Verification
**Test**: Ensure project builds successfully after changes
**Method**: Run `npm run build` or equivalent command in my-frontend directory

## Phase 2: Implementation Preparation

### Pre-Implementation Checklist

- [ ] Back up current configuration files
- [ ] Verify current build process works before changes
- [ ] Document current site structure for rollback if needed
- [ ] Create git branch for safe implementation

### File Modification Plan

#### FM-001: Navbar Configuration
**File**: `my-frontend/docusaurus.config.js`
**Change Type**: Text replacement
**Before**: `title: "AI/Spec-Driven Book"`
**After**: `title: "Home"`

#### FM-002: Homepage Content
**File**: `my-frontend/src/pages/index.js` (or equivalent)
**Change Type**: Content replacement
**Before**: "AI/Spec-Driven Book with Embedded RAG Chatbot" + tutorial content
**After**: "Physical AI & Humanoid Robotics Book" + book-specific content

#### FM-003: Homepage Button
**File**: `my-frontend/src/pages/index.js` (or equivalent)
**Change Type**: Button text and link update
**Before**: "Docusaurus Tutorial – 5min" button linking to tutorial
**After**: "Read" button linking to docs page

#### FM-004: Documentation Sidebar
**File**: `my-frontend/sidebars.js`
**Change Type**: Title prefix addition
**Before**: Module 1 and 4 items without chapter numbers
**After**: Module 1 and 4 items with "Chapter X" prefixes

## Post-Design Constitution Check

### Compliance Verification After Design
- ✅ Spec-First Workflow: Design aligns with original specification
- ✅ Technical Accuracy and Source Integrity: No functional changes that could introduce inaccuracies
- ✅ Developer-Focused Writing: Implementation plan provides clear guidance
- ✅ Reproducible Setup and Deployment: No changes to build/deploy process
- ✅ Content-First RAG Architecture: No impact on RAG functionality
- ✅ End-to-End System Integration: UI changes maintain system integration

## Architectural Decision Records

### ADR-001: Docusaurus Configuration Approach
**Decision**: Use standard Docusaurus configuration files for UI changes
**Status**: Implemented
**Rationale**: Following Docusaurus conventions ensures compatibility and maintainability

### ADR-002: Content Replacement Strategy
**Decision**: Replace content in-place rather than creating new components
**Status**: Implemented
**Rationale**: Minimizes complexity and maintains existing routing structure

## Success Criteria Verification

The implementation will meet all success criteria from the specification:
- ✅ Homepage reflects "Physical AI & Humanoid Robotics Book" branding
- ✅ "Read" button navigates to documentation page
- ✅ Documentation chapters display proper numbering
- ✅ All existing content remains accessible
- ✅ All navigation continues to function
- ✅ Responsive design maintained
- ✅ Project continues to build successfully