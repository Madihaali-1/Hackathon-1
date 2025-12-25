---
id: "6"
title: "Vercel Deployment Fix - Root Directory Configuration"
stage: "misc"
date: "2025-12-26"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "3-ai-robot-brain-isaac"
branch: "3-ai-robot-brain-isaac"
user: "user"
command: "Fix Vercel deployment error"
labels: ["vercel", "deployment", "docusaurus", "configuration"]
links:
  spec: "C:\\hackathon-1\\specs\\3-ai-robot-brain-isaac\\spec.md"
  plan: "C:\\hackathon-1\\specs\\3-ai-robot-brain-isaac\\plan.md"
  ticket: null
  adr: null
  pr: null
files:
  - "C:\\hackathon-1\\vercel.json"
tests: []
outcome: "Successfully configured Vercel to deploy Docusaurus site from my-frontend subdirectory by setting rootDirectory property."
evaluation: "Vercel configuration now properly handles deployment from subdirectory where Docusaurus project resides."
---

# Prompt History Record: Vercel Deployment Fix - Root Directory Configuration

## Original Issue

Vercel deployment was failing with the error: "The docs folder does not exist for version 'current'. A docs folder is expected to be found at docs."

The error occurred because:
- The Docusaurus project is located in the 'my-frontend' subdirectory
- Vercel was trying to build from the repository root
- Docusaurus couldn't find the docs folder at the expected location

## Solution Applied

Created vercel.json with the following configuration:

```json
{
  "rootDirectory": "my-frontend",
  "buildCommand": "npm run build",
  "devCommand": "npm run start"
}
```

This tells Vercel to:
- Treat the 'my-frontend' directory as the project root
- Run 'npm run build' in the my-frontend directory (which executes 'docusaurus build')
- Serve the generated static files from my-frontend/build

## Verification

- Confirmed that 'npm run build' works correctly in the my-frontend directory
- Verified that Docusaurus can find the docs folder when running from the correct directory
- Configured Vercel to properly handle the subdirectory structure

The deployment should now succeed with the Docusaurus site properly built and served.