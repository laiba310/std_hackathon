---
id: 001
title: Create Backend FastAPI Plan
stage: plan
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-backend-auth-chat
branch: 001-backend-auth-chat
user: User
command: /sp.plan Create a detailed implementation plan for the FastAPI Backend.

**Context:**
We are switching to **Google Gemini (Free)** and **Neon Postgres**.

**Phasing Strategy:**
1. **Phase 1: Foundation.**
   - Initialize `backend/` folder.
   - Create `requirements.txt` with: `fastapi`, `uvicorn`, `sqlalchemy`, `psycopg2-binary`, `python-jose[cryptography]`, `passlib[bcrypt]`, `python-multipart`, `google-generativeai`, `qdrant-client`, `fastembed`, `python-dotenv`.
   - Setup `.env` and CORS.

2. **Phase 2: Database & Auth.**
   - Setup Database connection (SQLAlchemy + Neon).
   - Create `users` table with `experience_level`.
   - Implement JWT Signup/Login.

3. **Phase 3: Translation Service.**
   - Integrate `google-generativeai` (Gemini Flash).
   - Create Protected Endpoint: `POST /api/translate`.

4. **Phase 4: RAG Chatbot.**
   - Setup Qdrant connection.
   - Create ingestion script (`ingest.py`).
   - Create Chat Endpoint: `POST /api/chat`.
labels: ["backend","fastapi","plan"]
links:
  spec: specs/001-backend-auth-chat/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/001-backend-auth-chat/plan.md
tests:
- null
---

## Prompt

Create a detailed implementation plan for the FastAPI Backend.

**Context:**
We are switching to **Google Gemini (Free)** and **Neon Postgres**.

**Phasing Strategy:**
1. **Phase 1: Foundation.**
   - Initialize `backend/` folder.
   - Create `requirements.txt` with: `fastapi`, `uvicorn`, `sqlalchemy`, `psycopg2-binary`, `python-jose[cryptography]`, `passlib[bcrypt]`, `python-multipart`, `google-generativeai`, `qdrant-client`, `fastembed`, `python-dotenv`.
   - Setup `.env` and CORS.

2. **Phase 2: Database & Auth.**
   - Setup Database connection (SQLAlchemy + Neon).
   - Create `users` table with `experience_level`.
   - Implement JWT Signup/Login.

3. **Phase 3: Translation Service.**
   - Integrate `google-generativeai` (Gemini Flash).
   - Create Protected Endpoint: `POST /api/translate`.

4. **Phase 4: RAG Chatbot.**
   - Setup Qdrant connection.
   - Create ingestion script (`ingest.py`).
   - Create Chat Endpoint: `POST /api/chat`.

## Response snapshot

A detailed implementation plan for the FastAPI Backend has been generated and saved to `specs/001-backend-auth-chat/plan.md`. This plan outlines the technical context, constitution check, project structure, and a phased strategy for implementing authentication, translation, and RAG chatbot features. A deviation regarding AI integration (Google Gemini instead of OpenAI) was noted, and an ADR suggestion was provided.

## Outcome

- ✅ Impact: Detailed implementation plan for FastAPI backend created.
- 🧪 Tests: N/A
- 📁 Files: specs/001-backend-auth-chat/plan.md
- 🔁 Next prompts: /sp.tasks
- 🧠 Reflection: Successfully created detailed implementation plan for the FastAPI backend, including phasing strategy and dependency management. Identified a deviation from constitution regarding AI integration and suggested an ADR.

## Evaluation notes (flywheel)

- Failure modes observed: PHR script execution failure.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure robust PHR creation, possibly by fixing the bash script or implementing a more robust agent-native fallback.
