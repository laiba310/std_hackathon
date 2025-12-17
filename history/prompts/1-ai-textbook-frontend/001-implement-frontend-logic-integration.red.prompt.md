---
id: 001
title: Implement Frontend Logic Integration
stage: red
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-ai-textbook-frontend
branch: 001-backend-auth-chat
user: unknown
command: /sp.implement
labels: ["frontend","integration","auth","chat","translation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/services/api.js
  - frontend/src/components/AuthModal.js
  - frontend/src/components/ChapterToolbar.js
  - frontend/src/components/ChapterContentWrapper.js
  - frontend/src/theme/MDXComponents.js
  - frontend/src/components/FloatingChatbot.js
  - frontend/src/pages/index.js
tests:
---

## Prompt

```
/sp.implement

Implement Task 5 (Frontend Logic Integration) from specs/1-ai-textbook-frontend/tasks.md.

**Directives:**
1.  **Create API Service (`frontend/src/services/api.js`):**
    - Define `API_URL = "http://localhost:8000/api"`.
    - Create functions: `login(email, password)`, `signup(data)`, `translate(text)`, `chat(question)`.
    - Handle **JWT Token**: Save token to `localStorage` on login, and send it in Headers (`Authorization: Bearer ...`) for Chat/Translation.

2.  **Create Auth Modal (`frontend/src/components/AuthModal.js`):**
    - A popup component with tabs for "Login" and "Signup".
    - Fields: Email, Password, Experience Level (for Signup).
    - On success: Save token and close modal.

3.  **Connect Toolbar (`frontend/src/components/ChapterToolbar.js`):**
    - Import `api.js`.
    - On "Translate" click: Call `api.translate()` -> Display Urdu text below the paragraph.
    - If API returns 401 (Unauthorized), open the `AuthModal`.

4.  **Connect Chatbot (`frontend/src/components/FloatingChatbot.js`):**
    - Create a chat interface (Input + Message List).
    - On Send: Call `api.chat()` -> Show AI response.
    - If user not logged in: Show "Please Login to Chat" button opening `AuthModal`.

**Validation:**
- User should be able to Login from the frontend.
- Translation should appear on the page.
```

## Response snapshot

Implemented the frontend logic integration as per Task 5 directives. This included creating `frontend/src/services/api.js` for API communication with JWT handling, `frontend/src/components/AuthModal.js` for user authentication (login/signup), integrating translation functionality into `frontend/src/components/ChapterToolbar.js` with unauthorized access handling, creating `frontend/src/components/FloatingChatbot.js` for chat functionality with authentication prompts, and integrating the chatbot into `frontend/src/pages/index.js`. A `ChapterContentWrapper` component was also created and integrated into `frontend/src/theme/MDXComponents.js` to facilitate passing chapter content to the toolbar.

## Outcome

- ✅ Impact: Implemented frontend logic for API integration, authentication, translation, and chatbot features.
- 🧪 Tests: No automated tests were run by the agent. Manual validation is expected.
- 📁 Files: 7 files were created or modified.
- 🔁 Next prompts: User validation of frontend functionality.
- 🧠 Reflection: Successfully integrated frontend components with backend API, including JWT handling. Had to adjust the ChapterToolbar integration due to MDX content parsing, leading to the creation of ChapterContentWrapper.

## Evaluation notes (flywheel)

- Failure modes observed: Attempted to use bash script create-phr.sh which is not compatible with Windows environment. Attempted to write to an existing file without reading it first.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure cross-platform compatibility for utility scripts, potentially using Python or native platform commands.