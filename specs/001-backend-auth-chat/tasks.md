# Feature Tasks: Backend for Physical AI Textbook

**Branch**: `001-backend-auth-chat` | **Date**: 2025-12-03 | **Plan**: [specs/001-backend-auth-chat/plan.md](specs/001-backend-auth-chat/plan.md)

## Summary

This document outlines the atomic coding tasks to implement the FastAPI backend, structured into phases as defined in the implementation plan. Each task includes clear acceptance criteria and references to relevant parts of the plan.

## Tasks

### Phase 1: Foundation

**Task**: 1.1 Initialize Backend Project Structure
**Description**: Create the `backend/` directory, set up `requirements.txt` with all specified dependencies, and create a basic `main.py` for FastAPI application entry point with CORS configuration.
**Acceptance Criteria**:
- [ ] `backend/` directory exists at the root level.
- [ ] `backend/requirements.txt` exists and contains: `fastapi`, `uvicorn`, `sqlalchemy`, `psycopg2-binary`, `python-jose[cryptography]`, `passlib[bcrypt]`, `python-multipart`, `google-generativeai`, `qdrant-client`, `fastembed`, `python-dotenv`.
- [ ] `pip install -r backend/requirements.txt` runs successfully.
- [ ] `backend/.env` file is created (can be empty initially).
- [ ] `backend/main.py` exists with a basic FastAPI app and CORS middleware configured.
**Dependencies**: None
**References**:
- Plan: [Phase 1: Foundation](specs/001-backend-auth-chat/plan.md#phase-1-foundation)
- Constitution: [Backend Technology Stack](.specify/memory/constitution.md#22-backend-the-intelligence-layer)
- Constitution: [Secrets Management](.specify/memory/constitution.md#41-coding-standards)

---

### Phase 2: Database & Authentication

**Task**: 2.1 Database Connection and User Model
**Description**: Set up the database connection to Neon Postgres using SQLAlchemy and define the `User` model.
**Acceptance Criteria**:
- [ ] `backend/src/core/database.py` exists and configures the SQLAlchemy engine and session for Neon Postgres.
- [ ] `backend/src/models/user.py` exists and defines the `User` SQLAlchemy model with `id`, `email`, `hashed_password`, and `experience_level` fields.
- [ ] The `experience_level` field correctly enforces "Beginner" or "Programmer" values.
- [ ] Database migrations (if applicable for initial table creation) can be run successfully.
**Dependencies**: 1.1
**References**:
- Plan: [Phase 2: Database & Authentication](specs/001-backend-auth-chat/plan.md#phase-2-database--authentication-bonus-points)
- Constitution: [Database (Relational): Neon](.specify/memory/constitution.md#22-backend-the-intelligence-layer)

**Task**: 2.2 JWT Utilities and Password Hashing
**Description**: Implement utility functions for password hashing (bcrypt) and JWT creation/validation.
**Acceptance Criteria**:
- [ ] `backend/src/core/security.py` exists.
- [ ] Functions for password hashing and verification using bcrypt are implemented.
- [ ] Functions for creating, encoding, and decoding JWTs using `python-jose` or `PyJWT` are implemented.
- [ ] A JWT secret key is loaded from `.env`.
- [ ] JWTs are configured to expire after 24 hours.
**Dependencies**: 1.1
**References**:
- Plan: [Phase 2: Database & Authentication](specs/001-backend-auth-chat/plan.md#phase-2-database--authentication-bonus-points)
- Constitution: [Secrets Management](.specify/memory/constitution.md#41-coding-standards)

**Task**: 2.3 User Signup Endpoint
**Description**: Implement the `POST /api/auth/signup` endpoint.
**Acceptance Criteria**:
- [ ] `POST /api/auth/signup` endpoint is defined in `backend/src/api/auth.py` (or similar).
- [ ] Endpoint accepts `email`, `password`, and `experience_level`.
- [ ] Passwords are hashed using bcrypt before storage.
- [ ] New user data (email, hashed password, experience level) is stored in Neon Postgres.
- [ ] Duplicate email registration returns an appropriate error.
**Dependencies**: 2.1, 2.2
**References**:
- Plan: [Phase 2: Database & Authentication](specs/001-backend-auth-chat/plan.md#phase-2-database--authentication-bonus-points)

**Task**: 2.4 User Login Endpoint
**Description**: Implement the `POST /api/auth/login` endpoint.
**Acceptance Criteria**:
- [ ] `POST /api/auth/login` endpoint is defined.
- [ ] Endpoint accepts `email` and `password`.
- [ ] Verifies provided password against stored hashed password.
- [ ] Issues a valid JWT upon successful login.
- [ ] Invalid credentials return an appropriate error (e.g., 401 Unauthorized).
**Dependencies**: 2.1, 2.2
**References**:
- Plan: [Phase 2: Database & Authentication](specs/001-backend-auth-chat/plan.md#phase-2-database--authentication-bonus-points)

---

### Phase 3: Translation Service

**Task**: 3.1 Gemini Translation Integration
**Description**: Set up the Google Gemini client and implement the translation logic.
**Acceptance Criteria**:
- [ ] `backend/src/services/ai_service.py` (or similar) exists.
- [ ] Google Gemini client is initialized with API keys from `.env`.
- [ ] A function to translate English text to Urdu using Gemini 1.5 Flash is implemented.
**Dependencies**: 1.1
**References**:
- Plan: [Phase 3: Translation Service](specs/001-backend-auth-chat/plan.md#phase-3-translation-service)
- Constitution: [AI Integration](.specify/memory/constitution.md#22-backend-the-intelligence-layer)

**Task**: 3.2 Protected Translation Endpoint
**Description**: Create the `POST /api/translate` endpoint and protect it with JWT authentication.
**Acceptance Criteria**:
- [ ] `POST /api/translate` endpoint is defined in `backend/src/api/translation.py` (or similar).
- [ ] Endpoint requires a valid JWT for access.
- [ ] Accepts English text as input.
- [ ] Returns Urdu translation generated by the Gemini service.
- [ ] Invalid or missing JWT results in a 401 Unauthorized error.
**Dependencies**: 2.2, 3.1
**References**:
- Plan: [Phase 3: Translation Service](specs/001-backend-auth-chat/plan.md#phase-3-translation-service)

---

### Phase 4: RAG Chatbot

**Task**: 4.1 Qdrant Connection and Ingestion Script
**Description**: Set up the Qdrant connection and develop the `ingest.py` script.
**Acceptance Criteria**:
- [ ] `backend/src/core/qdrant.py` (or similar) exists and configures connection to Qdrant Cloud.
- [ ] `backend/ingest.py` script exists at the root of the `backend/` directory.
- [ ] `ingest.py` reads Markdown files from `../frontend/docs/`.
- [ ] `ingest.py` uses `fastembed` for local embeddings.
- [ ] `ingest.py` uploads processed content and embeddings to Qdrant.
**Dependencies**: 1.1
**References**:
- Plan: [Phase 4: RAG Chatbot](specs/001-backend-auth-chat/plan.md#phase-4-rag-chatbot)
- Constitution: [Database (Vector): Qdrant Cloud](.specify/memory/constitution.md#22-backend-the-intelligence-layer)

**Task**: 4.2 Protected Chat Endpoint
**Description**: Create the `POST /api/chat` endpoint and protect it with JWT authentication, integrating Qdrant and Gemini for RAG.
**Acceptance Criteria**:
- [ ] `POST /api/chat` endpoint is defined in `backend/src/api/chat.py` (or similar).
- [ ] Endpoint requires a valid JWT for access.
- [ ] Accepts a user query as input.
- [ ] Performs similarity search in Qdrant for relevant book content using the query.
- [ ] Passes retrieved content and user query to Google Gemini to formulate an answer.
- [ ] Returns the generated answer to the user.
- [ ] Invalid or missing JWT results in a 401 Unauthorized error.
**Dependencies**: 2.2, 4.1, 3.1
**References**:
- Plan: [Phase 4: RAG Chatbot](specs/001-backend-auth-chat/plan.md#phase-4-rag-chatbot)
- Constitution: [RAG Chatbot](.specify/memory/constitution.md#31-the-bonus-point-features-mandatory)

## Definition of Done for all Tasks

- All acceptance criteria for the task are met.
- Code is implemented with Python type hints and docstrings.
- Relevant unit and integration tests are written and pass.
- No new security vulnerabilities are introduced.
- Code is committed with a conventional commit message.