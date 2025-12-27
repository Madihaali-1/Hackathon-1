# Data Model: Docusaurus UI Changes for Physical AI & Humanoid Robotics Book

**Feature**: 005-book-ui-changes
**Created**: 2025-12-28
**Status**: UI-only changes, no data model modifications

## Overview

This feature involves UI/text changes only, with no modifications to underlying data models or storage structures. All existing data models remain unchanged.

## UI Content Elements

### Homepage Content Structure
- **Main Title**: "Physical AI & Humanoid Robotics Book" (replaces "AI/Spec-Driven Book with Embedded RAG Chatbot")
- **Call-to-Action Button**: "Read" button (replaces "Docusaurus Tutorial – 5min")
- **Navigation Link**: "Home" (replaces "AI/Spec-Driven Book")

### Documentation Structure
- **Module 1 Chapters**: Prefixed with "Chapter 1", "Chapter 2", "Chapter 3"
- **Module 4 Chapters**: Prefixed with "Chapter 1", "Chapter 2", "Chapter 3"
- **Content**: Unchanged (only display titles modified)

## Frontend Components

### Component: Navbar
- **Location**: `my-frontend/docusaurus.config.js`
- **Property**: `title`
- **Type**: String
- **Value**: "Home"

### Component: Homepage
- **Location**: `my-frontend/src/pages/index.js` (or equivalent)
- **Properties**: Main title, CTA button
- **Type**: React component with text content
- **Values**: Updated to reflect book theme

### Component: Documentation Sidebar
- **Location**: `my-frontend/sidebars.js`
- **Properties**: Chapter titles in Modules 1 and 4
- **Type**: Configuration object with title strings
- **Values**: Prefixed with chapter numbers

## Constraints
- All existing content structure remains unchanged
- Only display text is modified, not underlying data
- Responsive design must be maintained
- No changes to data storage or retrieval