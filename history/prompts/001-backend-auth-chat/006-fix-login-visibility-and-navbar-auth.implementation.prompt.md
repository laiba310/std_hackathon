----
id: 006
title: Fix Login Visibility and Navbar Auth State
stage: implementation
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-backend-auth-chat
branch: 001-backend-auth-chat
user: PMLS
command: /sp.implement
labels: ["bugfix", "ui", "authentication"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/components/FloatingChatbot.js
  - frontend/src/theme/NavbarItem/AuthLink.js
  - frontend/docusaurus.config.js
  - frontend/src/components/AuthModal.js
tests:
null
---

## Prompt

Fix the critical Login Visibility issue and Navbar Login/Logout state management.

**Context:**
The current FloatingChatbot does not show the Login button in its header because the `isLoggedIn` check is flawed or the component update failed. The user needs a clear way to login.

**Directives:**

1. **Audit and Correct `frontend/src/components/FloatingChatbot.js`:**
   - **Ensure:** The `isLoggedIn` state relies purely on the presence of `localStorage.getItem('jwt_token')`.
   - **Fix:** Ensure the logic `{!isLoggedIn && (<Login Link/>)}` is rendered in the chat header, ensuring the Login link is always available when the token is missing.

2. **Implement Navbar Auth State:**
   - Create a new file `frontend/src/theme/NavbarItem/AuthLink.js`.
   - This component should check `localStorage.getItem('jwt_token')`.
   - **Logic:** If token exists, show a **"Logout"** button that clears `localStorage` and reloads. If token is missing, show the **"Login"** button that opens `AuthModal`.
   - **Update `docusaurus.config.js`:** Replace the existing GitHub link (or add a new item) to use this custom component.

3. **Update `AuthModal.js`:**
   - Ensure the `handleSubmit` function **clears localStorage** if the user clicks a Logout option (if we implement one, or just ensure the login sets it).

**Validation:**
- User must see a "Login" button in the Chat Header.
- User must see a "Login" link/button in the main Navbar.

## Response snapshot

All directives have been completed:
1. `frontend/src/components/FloatingChatbot.js` updated to use `isLoggedIn` state based on `localStorage.getItem('jwt_token')` and conditionally renders the Login link.
2. `frontend/src/theme/NavbarItem/AuthLink.js` created, handling Login/Logout state in the Navbar.
3. `frontend/docusaurus.config.js` updated to use the new `AuthLink` component in the Navbar.
4. `frontend/src/components/AuthModal.js` verified; the `login` function is responsible for setting the `jwt_token`, and logout is handled by `AuthLink.js`.

## Outcome

- ✅ Impact: Login visibility issues resolved in FloatingChatbot header and a clear Login/Logout mechanism implemented in the Navbar.
- 🧪 Tests: No automated tests were run; manual visual verification is required for both the chatbot header and Navbar.
- 📁 Files: Modified FloatingChatbot.js, created AuthLink.js, modified docusaurus.config.js, and verified AuthModal.js.
- 🔁 Next prompts: null
- 🧠 Reflection: The login state is now correctly managed across components, providing a consistent user experience.

## Evaluation notes (flywheel)

- Failure modes observed: Incorrect conditional rendering of login elements.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
