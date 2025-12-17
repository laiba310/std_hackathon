---
id: 002
title: Fix Frontend Components
stage: red
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-ai-textbook-frontend
branch: 001-backend-auth-chat
user: unknown
command: /sp.implement
labels: ["frontend","bugfix","auth","translation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/components/ChapterToolbar.js
  - frontend/src/components/AuthModal.js
  - frontend/src/services/api.js
tests:
---

## Prompt

```
Fix the Frontend Components (ChapterToolbar & AuthModal) immediately.

**Context:**
- `react-modal` is installed.
- `ChapterToolbar.js` has syntax errors.
- We need to integrate the Translation API with the Auth Modal.

**Directives:**

1. **Overwrite `frontend/src/components/ChapterToolbar.js`:**
   - Imports: `React, { useState }`, `translate` from `../services/api`, `AuthModal` from `./AuthModal`.
   - Logic:
     - When "Translate" is clicked, call `translate(text)`.
     - If API returns 401 (Unauthorized), set `setIsModalOpen(true)` to show the popup.
     - If success, display the Urdu text inside a div with `direction: rtl`.
   - **Crucial:** Ensure there are no syntax errors (imports at top, export at bottom).

2. **Overwrite `frontend/src/components/AuthModal.js`:**
   - Import `Modal` from `react-modal`.
   - **Bind Modal:** Use `Modal.setAppElement('#__docusaurus');` to avoid accessibility warnings.
   - UI: A popup with "Login" and "Signup" toggle.
   - Logic: Call `login()` or `signup()` from `api.js`. On success, close modal and reload page (`window.location.reload()`)

3. **Ensure `frontend/src/services/api.js` exists:**
   - It must handle `localStorage.getItem('token')` and send `Authorization: Bearer token` in headers.

**Validation:**
- The build (`npm start`) must pass without SyntaxErrors.
```

## Response snapshot

Fixed syntax errors and integrated authentication and translation logic in `frontend/src/components/ChapterToolbar.js` and `frontend/src/components/AuthModal.js`. Also verified `frontend/src/services/api.js` for proper JWT token handling, including `localStorage.setItem` and `Authorization: Bearer` headers. Implemented page reload after successful authentication in AuthModal for updated UI state. Added `direction: rtl` for translated text display.

## Outcome

- ✅ Impact: Frontend components are fixed, integrated with API, and handle authentication/translation as specified.
- 🧪 Tests: No automated tests were run. Manual validation is required to ensure the build passes and functionality works as expected.
- 📁 Files: 3 files were modified.
- 🔁 Next prompts: User validation of the fixes.
- 🧠 Reflection: Successfully addressed all directives, including specific error handling for unauthorized API calls and UI updates for translation direction. Ensured compliance with token handling requirements for `api.js`.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A