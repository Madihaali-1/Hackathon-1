# Data Model: ROS 2 Humanoid System Module

## Content Structure

### Chapter Entity
- **id**: Unique identifier for the chapter
- **title**: Display title of the chapter
- **description**: Brief description of the chapter content
- **content**: Markdown content of the chapter
- **order**: Numeric order for navigation
- **prerequisites**: List of required knowledge before reading this chapter
- **related**: List of related chapters or resources

### Navigation Entity
- **label**: Display name in sidebar
- **to**: Path to the document
- **order**: Order in the navigation hierarchy
- **children**: Sub-items under this navigation item

## Content Validation Rules

### Chapter Validation
- Each chapter must have a unique title
- Each chapter must have a valid order number
- Each chapter must contain educational content appropriate for the target audience
- Each chapter must include code examples where applicable
- Each chapter must link to related resources

### Navigation Validation
- Navigation items must be ordered logically
- Navigation items must have valid document paths
- Navigation hierarchy must not exceed 3 levels

## State Transitions

### Content States
- **Draft**: Initial state when chapter is created
- **Review**: Content is ready for review by subject matter experts
- **Approved**: Content has been reviewed and approved
- **Published**: Content is available to users

### Transition Rules
- Draft → Review: When content is substantially complete
- Review → Approved: After successful review by experts
- Approved → Published: When included in a release