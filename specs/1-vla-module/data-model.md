# Data Model: Module 4: Vision-Language-Action (VLA)

## Entities

### VLA Module
- **name**: Vision-Language-Action Module
- **description**: Educational unit covering VLA systems
- **components**: [Voice-to-Action, Cognitive Planning, Capstone Project]
- **status**: Published

### Voice-to-Action Chapter
- **title**: Voice-to-Action using OpenAI Whisper
- **content_type**: Educational documentation
- **dependencies**: OpenAI Whisper, Audio processing concepts
- **learning_objectives**: [Speech recognition, Action mapping, Integration patterns]

### Cognitive Planning Chapter
- **title**: Cognitive Planning with LLMs for ROS 2 Actions
- **content_type**: Educational documentation
- **dependencies**: LLMs, ROS 2, Planning algorithms
- **learning_objectives**: [LLM integration, ROS 2 actions, Planning strategies]

### Capstone Project Chapter
- **title**: Capstone Project: The Autonomous Humanoid
- **content_type**: Educational documentation
- **dependencies**: All previous chapters, Robotics concepts
- **learning_objectives**: [System integration, Multi-modal processing, Safety considerations]

### Docusaurus Documentation Site
- **type**: Static site generator
- **content**: Educational modules
- **navigation**: Hierarchical structure
- **integration_points**: Sidebar, search, cross-references

## Relationships
- VLA Module contains 3 chapters (Voice-to-Action, Cognitive Planning, Capstone Project)
- Each chapter has dependencies on specific technologies and concepts
- All chapters integrate with the Docusaurus documentation site
- Navigation structure connects all modules in the learning path