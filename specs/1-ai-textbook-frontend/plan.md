# Implementation Plan: Frontend for "Physical AI & Humanoid Robotics" Textbook

**Branch**: `1-ai-textbook-frontend` | **Date**: 2025-12-02 | **Spec**: specs/1-ai-textbook-frontend/spec.md
**Input**: Feature specification from `/specs/1-ai-textbook-frontend/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the implementation for the "Physical AI & Humanoid Robotics" Textbook Frontend, a Docusaurus-based static site. The core functionality includes a modern landing page, a structured documentation section following a defined curriculum, interactive UI components such as a "Translate to Urdu" button with a backend proxy, a "Personalize Content" dropdown with user database storage, and a floating AI Assistant chatbot for RAG-based Q&A. The site will also feature a custom blog layout with minor adjustments and a "Login/Signup" button integrating directly with a backend API for Better-Auth. The deployment will be configured for GitHub Pages via GitHub Actions.

## Technical Context

**Language/Version**: JavaScript (React), Docusaurus v3
**Primary Dependencies**: React, Docusaurus, potentially Axios/Fetch for API calls.
**Storage**: Frontend will interact with a backend user database for personalization state; static content is Markdown.
**Testing**: React Testing Library, Jest (for UI components). Docusaurus built-in testing (if any).
**Target Platform**: Web (Static Site, GitHub Pages).
**Project Type**: Web application (Docusaurus Static Site).
**Performance Goals**: Fast initial load times for static content, responsive UI for interactive elements (translation, personalization, chatbot).
**Constraints**:
*   Sidebar must strictly match the "Physical AI" curriculum structure.
*   The Orange/Black (Dark) and Orange/White (Light) themes must meet accessibility standards (WCAG 2.1 AA contrast ratios).
**Scale/Scope**: Educational textbook with a moderate user base, designed for future extensibility with AI features.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **AI-First Methodology:** Adhering to Spec-Kit Plus workflow. (PASS)
- **Reusable Intelligence:** Focus on reusable components (UI components like Toolbar, Chatbot). (PASS)
- **User-Centricity:** Personalization and localization features planned. (PASS)

### 2.1 Frontend (The Textbook Interface)
- **Framework:** Docusaurus (React-based) is mandatory. (PASS)
- **Hosting:** Deployable to GitHub Pages via GitHub Actions. (PASS)
- **Styling:** Use standard Docusaurus styling (Infima) to maintain a professional, academic aesthetic. (POTENTIAL CONFLICT - User specified custom Orange theme. This needs to be addressed.)
- **Authentication:** Better-Auth for user signup/signin, user sessions persist. (PASS - "Login/Signup" placeholder, direct API integration planned for Better-Auth.)

### 3.1 The "Bonus Point" Features (Mandatory)
- **Personalization Button:** Every chapter must have a button that rewrites content. (PASS)
- **Urdu Translation:** Every chapter must have a "Translate to Urdu" button. (PASS)
- **RAG Chatbot:** Floating chat interface globally available. (PASS)

## Project Structure

### Documentation (this feature)

```text
specs/1-ai-textbook-frontend/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend/ (or src/ as per Docusaurus default)
├── src/
│   ├── components/       # Toolbar, Chatbot, Custom Blog components
│   ├── pages/            # Landing page, custom Markdown pages
│   ├── theme/            # Docusaurus theme overrides, custom.css
│   └── docs/             # Markdown files for textbook modules
└── blog/                 # Blog Markdown files
```

**Structure Decision**: The project will leverage the default Docusaurus structure, using `src/theme` for custom styling and `src/components` for reusable UI elements. The `docs` and `blog` directories will contain the respective Markdown content.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Styling: Custom Orange Theme | To create a distinct brand identity separate from default Docusaurus blue, as per user requirement. | Standard Infima styling would not meet the brand identity goal. |
