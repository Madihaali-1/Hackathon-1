---
id: "8"
title: "Docusaurus Config Fix - Explicit Docs Configuration"
stage: "misc"
date: "2025-12-26"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "3-ai-robot-brain-isaac"
branch: "3-ai-robot-brain-isaac"
user: "user"
command: "Fix Docusaurus configuration for Vercel deployment"
labels: ["docusaurus", "vercel", "deployment", "configuration"]
links:
  spec: "C:\\hackathon-1\\specs\\3-ai-robot-brain-isaac\\spec.md"
  plan: "C:\\hackathon-1\\specs\\3-ai-robot-brain-isaac\\plan.md"
  ticket: null
  adr: null
  pr: null
files:
  - "C:\\hackathon-1\\my-frontend\\docusaurus.config.js"
tests: []
outcome: "Successfully updated Docusaurus configuration to explicitly define docs path and routeBasePath to resolve versioning-related build errors."
evaluation: "Docusaurus configuration now explicitly defines docs plugin settings to prevent versioning conflicts during Vercel deployment."
---

# Prompt History Record: Docusaurus Config Fix - Explicit Docs Configuration

## Original Issue

Vercel deployment was failing with the error: "The docs folder does not exist for version 'current'. A docs folder is expected to be found at docs."

This error typically occurs when Docusaurus' versioning system conflicts with the expected directory structure during the build process on Vercel's servers.

## Solution Applied

Updated the docusaurus.config.js file to explicitly configure the docs plugin with specific path and routeBasePath settings:

```javascript
docs: {
  sidebarPath: require.resolve('./sidebars.js'),
  // Explicitly configure the docs plugin to avoid versioning issues
  path: 'docs',
  routeBasePath: 'docs',
  // Please change this to your repo.
  // Remove this to remove the "edit this page" links.
  editUrl:
    'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
},
```

## Why This Works

- **Explicit path configuration**: Explicitly defines where Docusaurus should look for docs
- **routeBasePath setting**: Ensures proper routing for the docs section
- **Prevents versioning conflicts**: Explicit configuration helps avoid Docusaurus' automatic versioning behavior that might conflict with the directory structure during build

## Verification

- Confirmed that `npm run build` still works correctly after the configuration change
- The build completes successfully with static files generated in the "build" directory
- All existing functionality is preserved

This configuration should resolve the Vercel deployment issue by ensuring Docusaurus can properly locate the docs folder regardless of the build environment.