---
id: 004
title: Implement UI Components (Task 6)
stage: implement
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-ai-textbook-frontend
branch: main
user: PMLS
command: Execute "Task 6" (UI Components) immediately to fix the build crash.
labels: ["implementation", "frontend", "docusaurus", "ui"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/components/ChapterToolbar.js (created)
  - frontend/src/components/ChapterToolbar.module.css (created)
  - frontend/src/theme/MDXComponents.js (created)
  - frontend/src/components/FloatingChatbot.js (created)
  - frontend/src/components/FloatingChatbot.module.css (created)
tests:
  - Verify "Expected component ChapterToolbar to be defined" error is resolved.
  - Verify the Toolbar appears at the top of every chapter page.
---

## Prompt

```text
I want you to Execute "Task 6" (UI Components) immediately to fix the build crash.

**Directives:**

1. **Create the Toolbar Component:**
   - Create `frontend/src/components/ChapterToolbar.js`.
   - Code it as a simple React component returning a `div` with a standard Docusaurus styling (e.g., `alert alert--secondary`).
   - Inside, place two buttons:
     - `<button>Translate to Urdu</button>`
     - `<button>Personalize: Beginner</button>` (Mock dropdown).
   - Style it with an Orange border to match our theme.

2. **Register Globally (The Fix):**
   - Create the file `frontend/src/theme/MDXComponents.js`.
   - Add the following code EXACTLY (this maps the markdown tag to the React component):
     ```javascript
     import React from 'react';
     // Import the original mapper
     import MDXComponents from '@theme-original/MDXComponents';
     import ChapterToolbar from '@site/src/components/ChapterToolbar';

     export default {
       // Re-use the default mapping
       ...MDXComponents,
       // Map the "<ChapterToolbar />" tag to our component
       ChapterToolbar,
     };
     ```

3. **Create Chatbot Placeholder:**
   - Create `frontend/src/components/FloatingChatbot.js`.
   - Render a fixed position button (bottom-right) with an icon/text "AI Assistant".

**Validation:**
- The "Expected component ChapterToolbar to be defined" error MUST be resolved.
- The Toolbar must appear at the top of every chapter page.
```

## Response snapshot

Successfully created `ChapterToolbar.js` and `FloatingChatbot.js`, along with their respective CSS modules, and registered `ChapterToolbar` in `MDXComponents.js` to fix the build crash.

## Outcome

- ✅ Impact: All UI components for Task 6 created. The `ChapterToolbar` is registered globally, resolving the build crash.
- 🧪 Tests: Expected component definition issue resolved.
- 📁 Files: Created `frontend/src/components/ChapterToolbar.js`, `frontend/src/components/ChapterToolbar.module.css`, `frontend/src/theme/MDXComponents.js`, `frontend/src/components/FloatingChatbot.js`, `frontend/src/components/FloatingChatbot.module.css`.
- 🔁 Next prompts: None.
- 🧠 Reflection: The creation of the React components and their global registration through `MDXComponents.js` directly addresses the user's requirement to fix the build crash and implement the UI components for Task 6. The `TodoWrite` tool ensured all sub-tasks were tracked.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
