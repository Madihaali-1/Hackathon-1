# Feature Specification: Docusaurus UI Changes for Physical AI & Humanoid Robotics Book

**Feature Branch**: `005-book-ui-changes`
**Created**: 2025-12-28
**Status**: Draft
**Input**: User description: " inside `my-frontend` folder.
Requirement: UI/text changes only, fully responsive, no breaking changes.

1) Navbar
- Replace navbar title “AI/Spec-Driven Book” with “Home”.
- On homepage, replace text “AI/Spec-Driven Book” with
  Embedded RAG Chatbot" with: "Physical AI & Humanoid Robotics Book”.

2) Homepage Button
- Replace “Docusaurus Tutorial – 5min” button with “Read”.
- On click, navigate to Docs page.

3) Homepage Content
- Remove all default Docusaurus tutorial/promotional content.
- Add clean, minimal, book-related content (Physical AI Humanoid Robotics).
- Homepage should look like a technical book landing page.

4) Docs Chapters Numbering
- Keep all 4 modules and their content unchanged.
- In Module 1 and Module 4, add chapter numbers before titles:
  Chapter 1, Chapter 2, Chapter 3.
- Do not change docs content or structure.

Rules:
- Do not change docs text.
- Do not break routing.
- Ensure mobile + desktop responsiveness.
- Keep project build & deploy ready.

Apply all changes only in `my-frontend`."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Navigate to Book Content (Priority: P1)

As a visitor to the Physical AI & Humanoid Robotics Book website, I want to easily access the book content from the homepage so that I can start reading immediately.

**Why this priority**: This is the core user journey that delivers the primary value of the book website - allowing users to access the content.

**Independent Test**: Can be fully tested by visiting the homepage and clicking the "Read" button, which should navigate to the docs page where the book content is available.

**Acceptance Scenarios**:

1. **Given** I am on the homepage, **When** I click the "Read" button, **Then** I am navigated to the documentation page with the book content
2. **Given** I am on the homepage, **When** I click the "Home" link in the navbar, **Then** I am taken back to the homepage

---

### User Story 2 - View Book-Specific Branding (Priority: P1)

As a visitor to the website, I want to see clear branding that indicates this is about "Physical AI & Humanoid Robotics Book" rather than generic AI content, so that I understand the specific focus of the content.

**Why this priority**: Clear branding helps users understand the specific focus of the book and builds trust that the content is relevant to their interests.

**Independent Test**: Can be fully tested by visiting the homepage and verifying that the title and content reflect "Physical AI & Humanoid Robotics Book" rather than the old generic branding.

**Acceptance Scenarios**:

1. **Given** I am on the homepage, **When** I view the page, **Then** I see "Physical AI & Humanoid Robotics Book" as the main title
2. **Given** I am on any page of the website, **When** I view the navbar, **Then** I see "Home" as the main navigation link

---

### User Story 3 - Access Structured Book Content (Priority: P2)

As a reader of the book, I want to see clearly numbered chapters in the documentation so that I can follow the content in a logical, sequential order.

**Why this priority**: Numbered chapters provide better organization and help readers follow the logical progression of topics in the book.

**Independent Test**: Can be fully tested by navigating to Module 1 and Module 4 documentation pages and verifying that chapters are numbered as "Chapter 1", "Chapter 2", "Chapter 3".

**Acceptance Scenarios**:

1. **Given** I am viewing Module 1 documentation, **When** I look at the chapter titles, **Then** I see them prefixed with "Chapter 1", "Chapter 2", etc.
2. **Given** I am viewing Module 4 documentation, **When** I look at the chapter titles, **Then** I see them prefixed with "Chapter 1", "Chapter 2", "Chapter 3", etc.

---

### Edge Cases

- What happens when a user visits the site on a mobile device with a small screen?
- How does the site handle users with accessibility requirements using screen readers?
- What if a user bookmarks a specific documentation page and returns later?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display "Home" as the navbar title instead of "AI/Spec-Driven Book"
- **FR-002**: System MUST display "Physical AI & Humanoid Robotics Book" as the main title on the homepage instead of "AI/Spec-Driven Book with Embedded RAG Chatbot"
- **FR-003**: System MUST replace the "Docusaurus Tutorial – 5min" button with a "Read" button on the homepage
- **FR-004**: System MUST navigate users to the Docs page when clicking the "Read" button
- **FR-005**: System MUST remove all default Docusaurus tutorial/promotional content from the homepage
- **FR-006**: System MUST display clean, minimal, book-related content on the homepage that looks like a technical book landing page
- **FR-007**: System MUST add chapter numbers ("Chapter 1", "Chapter 2", "Chapter 3") before titles in Module 1 documentation
- **FR-008**: System MUST add chapter numbers ("Chapter 1", "Chapter 2", "Chapter 3") before titles in Module 4 documentation
- **FR-009**: System MUST preserve all existing documentation content and structure without changes
- **FR-010**: System MUST maintain all existing routing functionality without breaking links
- **FR-011**: System MUST ensure all UI changes are fully responsive on both mobile and desktop devices
- **FR-012**: System MUST maintain project build and deployment readiness after changes

### Key Entities *(include if feature involves data)*

- **Homepage Content**: The main landing page content that represents the book's marketing and entry point
- **Navigation Structure**: The navbar and page routing that allows users to navigate between homepage and documentation
- **Documentation Chapters**: The book content organized in modules with numbered chapters

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of homepage visitors can successfully navigate to the book content using the "Read" button within 10 seconds of landing
- **SC-002**: Homepage reflects "Physical AI & Humanoid Robotics Book" branding instead of generic AI content for 100% of visitors
- **SC-003**: Documentation chapters in Module 1 and Module 4 display proper numbering ("Chapter 1", "Chapter 2", etc.) for 100% of users
- **SC-004**: All existing documentation content remains accessible and unchanged for users (0% content loss)
- **SC-005**: All existing navigation and routing continues to function without breaking for 100% of users
- **SC-006**: Website displays correctly on both mobile and desktop devices for 100% of users
- **SC-007**: Project continues to build and deploy successfully after implementing changes (0% build failures)