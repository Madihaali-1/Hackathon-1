# Feature Specification: Docusaurus UI Text and Navigation Updates

**Feature Branch**: `004-docusaurus-ui-upgrade`
**Created**: 2025-12-27
**Status**: Draft
**Input**: User description: "You are a senior frontend engineer and Docusaurus expert.

Project: Docusaurus frontend inside `my-frontend` folder.
Requirement: UI/text changes only, fully responsive, no breaking changes.

1) Navbar
- Replace navbar title "AI/Spec-Driven Book" with "Home".
- On homepage, replace text "AI/Spec-Driven Book  with
  Embedded RAG Chatbot" with: Physical AI & Humanoid Robotics Book".

2) Homepage Button
- Replace "Docusaurus Tutorial – 5min" button with "Read".
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

### User Story 1 - Updated Navbar Experience (Priority: P1)

As a visitor to the Docusaurus site, I want to see a clear "Home" label in the navbar instead of "AI/Spec-Driven Book" so that I understand the navigation structure and can easily return to the homepage.

**Why this priority**: The navbar is the primary navigation element and changing it to "Home" provides clearer user experience and better aligns with user expectations.

**Independent Test**: When visiting any page on the site, the navbar title should display "Home" instead of the previous "AI/Spec-Driven Book" text.

**Acceptance Scenarios**:

1. **Given** I am on any page of the site, **When** I look at the navbar, **Then** I see "Home" as the title instead of "AI/Spec-Driven Book"
2. **Given** I am on the homepage, **When** I see the hero section text, **Then** I see "Physical AI & Humanoid Robotics Book" instead of "AI/Spec-Driven Book with Embedded RAG Chatbot"

---

### User Story 2 - Homepage Content and Navigation (Priority: P2)

As a visitor to the homepage, I want to see book-related content about Physical AI & Humanoid Robotics with a "Read" button that takes me to the documentation, so that I can easily access the book content.

**Why this priority**: The homepage is the entry point for users and should clearly represent the book's content and provide intuitive navigation to the documentation.

**Independent Test**: The homepage should display clean, minimal book-related content about Physical AI & Humanoid Robotics with a "Read" button that navigates to the Docs page when clicked.

**Acceptance Scenarios**:

1. **Given** I am on the homepage, **When** I view the content, **Then** I see clean, minimal book-related content about Physical AI & Humanoid Robotics instead of default Docusaurus tutorial content
2. **Given** I am on the homepage, **When** I click the "Read" button, **Then** I am navigated to the Docs page

---

### User Story 3 - Documentation Chapter Numbering (Priority: P3)

As a reader of the documentation, I want to see chapter numbers (Chapter 1, Chapter 2, Chapter 3) in Modules 1 and 4, so that I can better follow the book's structure and organization.

**Why this priority**: Chapter numbering provides better organization and helps readers understand the sequence and structure of the content.

**Independent Test**: In Module 1 and Module 4 of the documentation, chapter numbers should be displayed before the titles (Chapter 1, Chapter 2, Chapter 3) without changing the actual content.

**Acceptance Scenarios**:

1. **Given** I am viewing Module 1 documentation, **When** I look at the chapter titles, **Then** I see chapter numbers (Chapter 1, Chapter 2, Chapter 3) before the titles
2. **Given** I am viewing Module 4 documentation, **When** I look at the chapter titles, **Then** I see chapter numbers (Chapter 1, Chapter 2, Chapter 3) before the titles
3. **Given** I am viewing Module 2 or Module 3 documentation, **When** I look at the chapter titles, **Then** I see the titles without chapter numbers

---

### Edge Cases

- What happens when users resize the browser window to different sizes (mobile, tablet, desktop)?
- How does the site handle different screen orientations on mobile devices?
- What if users have accessibility requirements (screen readers, high contrast, etc.)?
- How does the navigation behave when JavaScript is disabled?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST replace the navbar title "AI/Spec-Driven Book" with "Home" across all pages
- **FR-002**: System MUST update the homepage hero section text from "AI/Spec-Driven Book with Embedded RAG Chatbot" to "Physical AI & Humanoid Robotics Book"
- **FR-003**: System MUST replace the "Docusaurus Tutorial – 5min" button with a "Read" button on the homepage
- **FR-004**: System MUST ensure the "Read" button navigates to the Docs page when clicked
- **FR-005**: System MUST remove all default Docusaurus tutorial/promotional content from the homepage
- **FR-006**: System MUST add clean, minimal book-related content about Physical AI & Humanoid Robotics to the homepage
- **FR-007**: System MUST format the homepage to look like a technical book landing page
- **FR-008**: System MUST add chapter numbers (Chapter 1, Chapter 2, Chapter 3) before titles in Module 1 documentation
- **FR-009**: System MUST add chapter numbers (Chapter 1, Chapter 2, Chapter 3) before titles in Module 4 documentation
- **FR-010**: System MUST NOT change the actual documentation content or structure in any modules
- **FR-011**: System MUST maintain all 4 existing modules and their content unchanged
- **FR-012**: System MUST preserve routing and navigation functionality across the site
- **FR-013**: System MUST ensure all changes are fully responsive on mobile and desktop devices
- **FR-014**: System MUST maintain build and deploy readiness of the project
- **FR-015**: System MUST apply all changes only within the `my-frontend` directory

### Key Entities

- **Navbar Element**: Navigation bar component that displays the site title and requires text update
- **Homepage Content**: Hero section and promotional content that needs replacement with book-related information
- **Navigation Button**: Call-to-action button that needs text change and updated navigation behavior
- **Documentation Modules**: Four existing modules where specific modules (1 and 4) require chapter numbering
- **Responsive Layout**: Layout structure that must maintain compatibility across desktop, tablet, and mobile devices

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Navbar title successfully displays "Home" instead of "AI/Spec-Driven Book" on all pages
- **SC-002**: Homepage hero section text displays "Physical AI & Humanoid Robotics Book" instead of "AI/Spec-Driven Book with Embedded RAG Chatbot"
- **SC-003**: Homepage button text successfully changes from "Docusaurus Tutorial – 5min" to "Read" and navigates to Docs page when clicked
- **SC-004**: Homepage displays clean, minimal book-related content about Physical AI & Humanoid Robotics without default Docusaurus promotional content
- **SC-005**: Homepage has a technical book landing page appearance with appropriate styling and layout
- **SC-006**: Module 1 documentation displays chapter numbers (Chapter 1, Chapter 2, Chapter 3) before titles
- **SC-007**: Module 4 documentation displays chapter numbers (Chapter 1, Chapter 2, Chapter 3) before titles
- **SC-008**: Modules 2 and 3 documentation titles remain unchanged without chapter numbers
- **SC-009**: All existing documentation content and structure remains unchanged in all modules
- **SC-010**: All navigation and routing functionality remains intact and operational
- **SC-011**: Site maintains full responsiveness across mobile, tablet, and desktop devices
- **SC-012**: Project builds and deploys successfully without errors after all changes
- **SC-013**: All changes are contained within the `my-frontend` directory without affecting other project components
