----
id: 004
title: Refactor FloatingChatbot for Modern UI and Login Visibility
stage: refactor
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-backend-auth-chat
branch: 001-backend-auth-chat
user: PMLS
command: /sp.implement
labels: ["ui", "refactor", "chatbot", "login"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/components/FloatingChatbot.module.css
  - frontend/src/components/FloatingChatbot.js
  - package.json
  - package-lock.json
tests:
null
---

## Prompt

Refactor the FloatingChatbot component to have a modern, beautiful ("pyara") UI and fix the login visibility.

**Context:**
The current chatbot look is too basic and the robot icon is not appealing. The user is also facing issues finding the login option. We need a polished, animated UI that matches our Orange/Black robotics theme and makes Login easily accessible.

**Directives:**

1. **Install Dependencies:**
   - Run `npm install react-icons` in the `frontend` directory (if not already installed).

2. **Overwrite `frontend/src/components/FloatingChatbot.module.css` with this Modern CSS:**
   ```css
   /* Main Container - Bottom Right */
   .chatbotContainer {
     position: fixed;
     bottom: 25px;
     right: 25px;
     z-index: 1000;
     display: flex;
     flex-direction: column;
     align-items: flex-end;
     font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
   }

   /* The Floating Button - Modern & Soft */
   .toggleButton {
     background: linear-gradient(135deg, #FF8C00, #FF5F00); /* Softer Orange Gradient */
     color: white;
     border: none;
     border-radius: 50%;
     width: 64px;
     height: 64px;
     font-size: 26px;
     cursor: pointer;
     box-shadow: 0 6px 25px rgba(255, 95, 0, 0.5);
     transition: all 0.3s cubic-bezier(0.25, 0.8, 0.25, 1);
     display: flex;
     align-items: center;
     justify-content: center;
   }

   .toggleButton:hover {
     transform: scale(1.08) translateY(-3px);
     box-shadow: 0 10px 30px rgba(255, 95, 0, 0.7);
   }

   /* Chat Window - Clean Card Design */
   .chatWindow {
     width: 360px;
     height: 550px;
     background-color: var(--ifm-background-color);
     border-radius: 20px;
     box-shadow: 0 12px 40px rgba(0, 0, 0, 0.15);
     display: flex;
     flex-direction: column;
     margin-bottom: 18px;
     overflow: hidden;
     border: 1px solid rgba(0,0,0,0.05);
     animation: slideIn 0.35s cubic-bezier(0.25, 0.8, 0.25, 1);
     transform-origin: bottom right;
   }

   @keyframes slideIn {
     from { opacity: 0; transform: scale(0.9) translateY(20px); }
     to { opacity: 1; transform: scale(1) translateY(0); }
   }

   /* Header - Clean & Informative */
   .chatHeader {
     background: linear-gradient(135deg, #FF8C00, #FF5F00);
     color: white;
     padding: 18px;
     display: flex;
     justify-content: space-between;
     align-items: center;
   }

   .headerTitle {
     font-weight: 700;
     font-size: 1.1rem;
     display: flex;
     align-items: center;
     gap: 10px;
   }

   .headerControls {
     display: flex;
     align-items: center;
     gap: 15px;
   }

   .loginLink {
       font-size: 0.9rem;
       cursor: pointer;
       text-decoration: underline;
       opacity: 0.9;
       background: rgba(255,255,255,0.2);
       padding: 4px 10px;
       border-radius: 12px;
   }
   .loginLink:hover { opacity: 1; background: rgba(255,255,255,0.3); }

   /* Messages Area - Clean Background */
   .messages {
     flex: 1;
     padding: 20px;
     overflow-y: auto;
     display: flex;
     flex-direction: column;
     gap: 15px;
     background-color: var(--ifm-background-surface-color);
     background-image: radial-gradient(var(--ifm-color-emphasis-200) 1px, transparent 1px);
     background-size: 20px 20px; /* Subtle dot pattern */
   }

   /* Modern Message Bubbles */
   .message {
     max-width: 85%;
     padding: 14px 18px;
     border-radius: 20px;
     font-size: 15px;
     line-height: 1.5;
     position: relative;
     word-wrap: break-word;
     box-shadow: 0 2px 5px rgba(0,0,0,0.05);
   }

   .user {
     align-self: flex-end;
     background: linear-gradient(135deg, #FF8C00, #FF5F00);
     color: white;
     border-bottom-right-radius: 4px;
   }

   .ai {
     align-self: flex-start;
     background-color: var(--ifm-background-color);
     color: var(--ifm-font-color-base);
     border: 1px solid var(--ifm-color-emphasis-200);
     border-bottom-left-radius: 4px;
   }

   /* Input Area - Floating capsule style */
   .inputArea {
     padding: 15px;
     background-color: var(--ifm-background-surface-color);
     border-top: 1px solid var(--ifm-color-emphasis-200);
     display: flex;
     align-items: center;
     gap: 10px;
   }

   .inputWrapper {
       flex: 1;
       background: var(--ifm-background-color);
       border: 2px solid var(--ifm-color-emphasis-300);
       border-radius: 30px;
       padding: 5px 15px;
       display: flex;
       align-items: center;
       transition: border-color 0.2s;
   }
   .inputWrapper:focus-within {
       border-color: #FF8C00;
   }

   .inputWrapper input {
     flex: 1;
     border: none;
     background: transparent;
     outline: none;
     padding: 8px;
     font-size: 15px;
     color: var(--ifm-font-color-base);
   }

   .sendButton {
     background: linear-gradient(135deg, #FF8C00, #FF5F00);
     color: white;
     border: none;
     padding: 12px;
     border-radius: 50%;
     width: 48px;
     height: 48px;
     cursor: pointer;
     transition: transform 0.2s, box-shadow 0.2s;
     display: flex;
     align-items: center;
     justify-content: center;
     box-shadow: 0 4px 10px rgba(255, 95, 0, 0.3);
   }

   .sendButton:hover:not(:disabled) {
     transform: scale(1.05);
     box-shadow: 0 6px 15px rgba(255, 95, 0, 0.4);
   }
   .sendButton:disabled {
       background: var(--ifm-color-emphasis-300);
       box-shadow: none;
       cursor: default;
   }
```

## Response snapshot

All directives have been completed:
1. `react-icons` checked and confirmed installed.
2. `frontend/src/components/FloatingChatbot.module.css` overwritten with modern CSS.
3. `frontend/src/components/FloatingChatbot.js` modified for new UI and login visibility, including icon changes and header/input area updates.

## Outcome

- ✅ Impact: Implemented new modern UI aesthetic and improved login visibility for FloatingChatbot component.
- 🧪 Tests: No specific tests were run for this UI refactor; visual inspection is required.
- 📁 Files: Updated CSS and JS component to align with new aesthetic and functionality.
- 🔁 Next prompts: null
- 🧠 Reflection: Successfully updated UI and login mechanism, ensuring a consistent and improved user experience. The component structure supported the required changes well.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
