---
id: "7"
title: "Vercel Deployment Config Updated - Correct Location"
stage: "misc"
date: "2025-12-26"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "3-ai-robot-brain-isaac"
branch: "3-ai-robot-brain-isaac"
user: "user"
command: "Update Vercel deployment configuration"
labels: ["vercel", "deployment", "docusaurus", "configuration"]
links:
  spec: "C:\\hackathon-1\\specs\\3-ai-robot-brain-isaac\\spec.md"
  plan: "C:\\hackathon-1\\specs\\3-ai-robot-brain-isaac\\plan.md"
  ticket: null
  adr: null
  pr: null
files:
  - "C:\\hackathon-1\\my-frontend\\vercel.json"
tests: []
outcome: "Successfully configured Vercel to deploy Docusaurus site by placing vercel.json in the correct directory (my-frontend)."
evaluation: "Vercel configuration now properly handles deployment when my-frontend is deployed as the root directory."
---

# Prompt History Record: Vercel Deployment Config Updated - Correct Location

## Original Issue

When deploying the my-frontend folder to Vercel (which contains the Docusaurus project), the build was failing because the vercel.json was in the wrong location.

## Solution Applied

1. Removed the vercel.json from the repository root
2. Created a new vercel.json in the my-frontend directory (where the Docusaurus project resides)

The new vercel.json contains:
```json
{
  "buildCommand": "npm run build",
  "devCommand": "npm run start"
}
```

## Why This Works

- When deploying the my-frontend folder to Vercel, that folder becomes the root directory
- The vercel.json should be in the root of the project being deployed
- The build command runs `npm run build` which executes `docusaurus build`
- Docusaurus can now properly find the docs folder since it's running from the correct directory

## Verification

- Confirmed that `npm run build` works correctly in the my-frontend directory
- The Docusaurus configuration is correct and can find the docs folder
- Vercel configuration is now properly set up for deployment

The deployment should now succeed with the Docusaurus site properly built and served.