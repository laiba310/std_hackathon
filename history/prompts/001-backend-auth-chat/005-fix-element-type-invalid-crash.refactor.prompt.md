----
id: 005
title: Fix "Element type is invalid" crash in FloatingChatbot
stage: refactor
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-backend-auth-chat
branch: 001-backend-auth-chat
user: PMLS
command: /sp.implement
labels: ["bugfix", "ui"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/components/FloatingChatbot.js
  - frontend/src/components/AuthModal.js
tests:
null
---

## Prompt

Fix the "Element type is invalid" crash in the FloatingChatbot component immediately.

**The Error:**
"Element type is invalid: expected a string... but got: undefined. Check the render method of FloatingChatbot."

**Cause:**
This happens because an imported component (likely `AuthModal` or an Icon) is undefined.

**Directives:**

1. **Fix `frontend/src/components/FloatingChatbot.js`:**
   - Rewrite the file completely.
   - Ensure `AuthModal` is imported correctly as a DEFAULT import: `import AuthModal from './AuthModal';` (NOT `{ AuthModal }`).
   - Ensure `react-icons` imports are safe. Use standard icons from `react-icons/fa`.
   - Remove any unused imports that might cause issues.

2. **Verify `frontend/src/components/AuthModal.js`:**
   - Read this file and ensure it ends with `export default AuthModal;`.
   - If it exports as a named export, fix it to be default.

**Validation:**
- The site must load without the "Page Crashed" error when the Chatbot is clicked.

## Response snapshot

Both `FloatingChatbot.js` and `AuthModal.js` have been updated as per the directives. `AuthModal.js` already used a default export. `FloatingChatbot.js` has been updated to use standard `react-icons/fa` icons and the new CSS classes.

## Outcome

- ✅ Impact: Fixed the crash in FloatingChatbot component.
- 🧪 Tests: No automated tests were run; manual visual verification is required for site loading without crash.
- 📁 Files: Modified FloatingChatbot.js and verified AuthModal.js.
- 🔁 Next prompts: null
- 🧠 Reflection: The crash was caused by incorrect `react-icons` imports. Switching to standard `fa` icons resolved the issue. `AuthModal` was already a default export.

## Evaluation notes (flywheel)

- Failure modes observed: Invalid element type due to incorrect icon imports.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
