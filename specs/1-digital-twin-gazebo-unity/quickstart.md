# Quickstart Guide: Module 2 - Digital Twin (Gazebo & Unity)

## Overview
This guide will help you set up and work with the Digital Twin module covering Gazebo physics simulation and Unity digital twins for humanoid robotics.

## Prerequisites
- Node.js >= 20.0 installed
- Git installed
- Basic understanding of robotics concepts (covered in Module 1)

## Setup Development Environment

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
cd my-frontend
npm install
```

### 3. Start Development Server
```bash
npm start
```
The site will be available at http://localhost:3000

## Adding Module 2 Content

### 1. Create Chapter Directories
```bash
mkdir -p docs/modules/digital-twin-gazebo-unity/chapter-1-physics-simulation
mkdir -p docs/modules/digital-twin-gazebo-unity/chapter-2-digital-twins-hri
mkdir -p docs/modules/digital-twin-gazebo-unity/chapter-3-sensor-simulation
```

### 2. Add Content Files
Create the following files with your chapter content:
- `docs/modules/digital-twin-gazebo-unity/chapter-1-physics-simulation/intro-to-gazebo.md`
- `docs/modules/digital-twin-gazebo-unity/chapter-2-digital-twins-hri/unity-digital-twins.md`
- `docs/modules/digital-twin-gazebo-unity/chapter-3-sensor-simulation/lidar-simulation.md`

### 3. Update Sidebar Navigation
Update `my-frontend/sidebars.js` to include the new module content.

## Running the Documentation Site

### Development Mode
```bash
cd my-frontend
npm start
```

### Build for Production
```bash
cd my-frontend
npm run build
```

### Serve Built Site Locally
```bash
cd my-frontend
npm run serve
```

## Content Creation Guidelines

### Markdown Structure
Each chapter should follow this structure:
```markdown
---
sidebar_position: [number]
---

# Chapter Title

## Learning Objectives
- Objective 1
- Objective 2

## Introduction
[Chapter introduction content]

## Section 1
[Section content with examples]

## Exercises
[Practice problems for students]

## Summary
[Chapter summary and next steps]
```

### Adding Code Examples
Use Docusaurus code blocks with appropriate language identifiers:
```python
# Python code example
def simulation_function():
    pass
```

```xml
<!-- XML configuration example -->
<sdf version="1.7">
  <!-- simulation configuration -->
</sdf>
```

### Adding Figures
Place images in `static/img/` directory and reference them:
```markdown
![Description of image](/img/path-to-image.png)
```

## Testing Your Changes

### 1. Verify Navigation
- Check that all chapters appear in the sidebar
- Verify links between sections work correctly

### 2. Validate Content
- Ensure all code examples render correctly
- Verify that all images display properly
- Test all external links

### 3. Check Cross-References
- Verify links to other modules work
- Ensure consistent styling with existing content

## Deployment
When your content is ready, commit your changes and push to the repository. The site will be automatically deployed to GitHub Pages if configured with GitHub Actions.