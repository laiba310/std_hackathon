# Implementation Plan: FastAPI Backend for Physical AI Textbook

**Branch**: `001-backend-auth-chat` | **Date**: 2025-12-03 | **Spec**: [specs/001-backend-auth-chat/spec.md](specs/001-backend-auth-chat/spec.md)
**Input**: Feature specification from `/specs/001-backend-auth-chat/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the implementation of a FastAPI backend for the Physical AI Textbook, focusing on user authentication with JWT, a protected RAG chatbot, a translation service, and an ingestion script. The technical approach leverages FastAPI, Neon Postgres, Qdrant, and Google Gemini, following a phased development strategy.

## Technical Context

**Language/Version**: Python 3.12+
**Primary Dependencies**: `fastapi`, `uvicorn`, `sqlalchemy`, `psycopg2-binary`, `python-jose[cryptography]`, `passlib[bcrypt]`, `python-multipart`, `google-generativeai`, `qdrant-client`, `fastembed`, `python-dotenv`
**Storage**: Neon (Serverless Postgres) for user profiles, Qdrant Cloud for vector database
**Testing**: pytest
**Target Platform**: Linux server
**Project Type**: Web application (Backend component)
**Performance Goals**: NEEDS CLARIFICATION (No specific performance goals were outlined in the specification or plan. Standard web API response times are assumed for now.)
**Constraints**: API keys must be loaded from `.env` files. JWTs will expire after 24 hours. Passwords will be hashed using bcrypt.
**Scale/Scope**: NEEDS CLARIFICATION (No specific user scale or data volume was outlined in the specification or plan. Assumed to handle a moderate number of users and book content.)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **AI-First Methodology:** Pass. Adhering to Spec-Kit Plus workflow.
- **Reusable Intelligence:** Pass. Future consideration for subagents and skills.
- **User-Centricity:** Pass. Personalization features will be built upon this backend.
- **Backend Language (Python 3.12+):** Pass.
- **API Framework (FastAPI):** Pass.
- **Database (Relational - Neon):** Pass.
- **Database (Vector - Qdrant Cloud):** Pass.
- **AI Integration (OpenAI API vs. Google Gemini):** Deviation. The plan explicitly uses Google Gemini as per user instruction. This is noted for an ADR.
- **Secrets Management (.env):** Pass.
- **Python Coding Standards (Type Hints, Docstrings):** Pass. Will be enforced during implementation.
- **Error Handling (Structured HTTP errors):** Pass. Will be enforced during implementation.

## Project Structure

### Documentation (this feature)

```text
specs/001-backend-auth-chat/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/          # SQLAlchemy models for database
│   ├── services/        # Business logic, AI integrations, Qdrant interaction
│   ├── api/             # FastAPI routers and endpoints
│   └── core/            # Configuration, CORS, JWT utilities, dependencies
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── .env                 # Environment variables
├── requirements.txt     # Python dependencies
└── main.py              # Main FastAPI application entry point

frontend/                # Existing frontend directory
├── docs/                # Markdown files for ingestion
└── ...
```

**Structure Decision**: The chosen structure is a standard web application layout with a dedicated `backend/` directory, reflecting the separation of concerns for the API, services, models, and core utilities. This aligns with common FastAPI project conventions and the overall project requirements for a distinct backend.

## Phasing Strategy

### Phase 1: Foundation
- **Objective**: Establish the basic backend project structure and essential configurations.
- **Steps**:
    1. Create the `backend/` directory at the root level.
    2. Initialize `requirements.txt` within `backend/` with specified dependencies (`fastapi`, `uvicorn`, `sqlalchemy`, `psycopg2-binary`, `python-jose[cryptography]`, `passlib[bcrypt]`, `python-multipart`, `google-generativeai`, `qdrant-client`, `fastembed`, `python-dotenv`).
    3. Set up `.env` file for environment variables (e.g., database connection string, JWT secret key, API keys).
    4. Configure CORS settings for the FastAPI application to allow communication with the frontend.

### Phase 2: Database & Authentication (Bonus Points)
- **Objective**: Implement user account management and JWT-based authentication.
- **Steps**:
    1. Establish connection to Neon Postgres using SQLAlchemy.
    2. Define a SQLAlchemy model for the `users` table, including `email`, `password` (hashed), and `experience_level` fields.
    3. Implement user signup (`POST /api/auth/signup`) endpoint to create new user accounts, hash passwords using bcrypt, and store user data in Neon Postgres.
    4. Implement user login (`POST /api/auth/login`) endpoint to authenticate users, verify passwords, and issue a JWT upon successful login.
    5. Develop JWT utility functions for token creation, encoding, and decoding using `python-jose[cryptography]` or `PyJWT`.
    6. Create a dependency injection for JWT validation to protect API endpoints.

### Phase 3: Translation Service
- **Objective**: Integrate the Gemini API for English to Urdu translation.
- **Steps**:
    1. Set up `google-generativeai` client with API keys from `.env`.
    2. Implement the translation logic using Gemini 1.5 Flash.
    3. Create a protected endpoint `POST /api/translate` that accepts English text and returns Urdu translation. This endpoint will require a valid JWT for access, aligning with the clarification provided.

### Phase 4: RAG Chatbot
- **Objective**: Develop the RAG chatbot functionality and content ingestion mechanism.
- **Steps**:
    1. Configure connection to Qdrant Cloud.
    2. Develop `ingest.py` script to:
        a. Read Markdown files from `../frontend/docs/`.
        b. Use `fastembed` locally to generate embeddings for the content.
        c. Upload the content and their embeddings to Qdrant.
    3. Implement the chat endpoint `POST /api/chat`.
        a. This endpoint must be protected by JWT authentication.
        b. On a valid request, use the user's query to perform a similarity search in Qdrant for relevant book content.
        c. Pass the retrieved content and the user's query to Google Gemini to formulate a coherent answer.
        d. Return the generated answer to the user.

## Research & Clarification (Phase 0)

No outstanding `NEEDS CLARIFICATION` items remain from the specification. All previous clarifications have been resolved. Further research tasks will be generated as needed during task execution.

## Data Model (Phase 1 - To be created)

This section will be detailed in `data-model.md` and will describe the entities like `User` and `BookContent` with their attributes and relationships, particularly for the Neon Postgres schema.

## API Contracts (Phase 1 - To be created)

This section will be detailed in the `contracts/` directory and will contain OpenAPI schemas for the authentication, chat, and translation APIs.

## Quickstart Guide (Phase 1 - To be created)

This section will be detailed in `quickstart.md` and will provide instructions for setting up and running the backend, including environment setup and basic API usage.

## Follow-ups and Risks

- **Risk**: Potential performance bottlenecks with large volumes of book content for RAG or high concurrent user requests to AI APIs. **Mitigation**: Implement caching strategies and monitor API usage.
- **Risk**: Ensuring robust security for JWTs (e.g., secure storage on the client side, token refresh mechanisms). **Mitigation**: Adhere to industry best practices for JWT handling and consider refresh tokens.
- **Follow-up**: Implement comprehensive unit and integration tests for all backend components, especially authentication and AI integrations.

This plan sets the stage for the backend development. The next step will be to create the `tasks.md` using the `/sp.tasks` command based on this plan.
