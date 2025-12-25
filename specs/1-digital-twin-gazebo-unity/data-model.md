# Data Model: Module 2 - Digital Twin (Gazebo & Unity)

## Core Entities

### Chapter
- **title**: String - The chapter title
- **description**: String - Brief description of the chapter content
- **learning_objectives**: Array<String> - What students will learn
- **prerequisites**: Array<String> - Required knowledge before starting
- **sections**: Array<Section> - List of sections in the chapter
- **exercises**: Array<Exercise> - Practice problems and assignments
- **resources**: Array<Resource> - Additional materials and references

### Section
- **title**: String - The section title
- **content**: String - The main content in Markdown format
- **examples**: Array<CodeExample> - Code or simulation examples
- **figures**: Array<Figure> - Images, diagrams, or screenshots
- **duration**: Number - Estimated time to complete (minutes)

### CodeExample
- **title**: String - Description of the example
- **language**: String - Programming language or format (bash, xml, python, etc.)
- **code**: String - The actual code or configuration
- **explanation**: String - Explanation of the example
- **expected_output**: String - What the example should produce

### Exercise
- **title**: String - Exercise title
- **description**: String - Detailed problem statement
- **difficulty**: String - Level (beginner, intermediate, advanced)
- **instructions**: Array<String> - Step-by-step instructions
- **expected_outcome**: String - What the solution should accomplish
- **solution**: String - Reference solution (optional, for instructors)

### Resource
- **title**: String - Resource title
- **url**: String - Link to external resource
- **type**: String - Type of resource (documentation, video, paper, etc.)
- **description**: String - Brief description of the resource value

### Figure
- **title**: String - Figure title
- **description**: String - Description of the figure content
- **src**: String - Path to the image file
- **alt_text**: String - Alternative text for accessibility
- **caption**: String - Caption for the figure

## Relationships

### Chapter contains Sections
- One Chapter contains many Sections (1 to many)

### Section contains Examples and Figures
- One Section contains many CodeExamples and Figures (1 to many)

### Chapter contains Exercises and Resources
- One Chapter contains many Exercises and Resources (1 to many)

## Validation Rules

### Chapter Validation
- Title must be 5-100 characters
- Description must be 20-500 characters
- Must have at least one section
- Learning objectives must be specific and measurable

### Section Validation
- Title must be 5-100 characters
- Content must be in valid Markdown format
- Duration must be between 5-120 minutes

### CodeExample Validation
- Language must be a recognized syntax highlighting language
- Code must be syntactically valid for the specified language
- Explanation must be provided for all examples

### Exercise Validation
- Difficulty must be one of: beginner, intermediate, advanced
- Instructions must be numbered and clear
- Expected outcome must be specific and testable