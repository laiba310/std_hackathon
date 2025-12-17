# Feature Specification: Backend for Physical AI Textbook

**Feature Branch**: `001-backend-auth-chat`
**Created**: 2025-12-03
**Status**: Draft
**Input**: User description: "Create the Backend Specification for "Physical AI Textbook".

**Project Structure:**
- Create a directory `backend/` at root.

**Core Responsibilities:**
1.  **Authentication System (Critical - 50 pts):**
    - **Endpoint:** `POST /api/auth/signup` & `POST /api/auth/login`.
    - **Logic:** Issue a **JWT (JSON Web Token)** upon successful login.
    - **Data:** Signup must require `email`, `password`, and `experience_level` ("Beginner"/"Programmer").
    - **Storage:** User data goes into **Neon Postgres**.

2.  **Protected Chat API (The Auth Wall):** `POST /api/chat`
    - **Security Rule:** This endpoint must be **PROTECTED**.
    - **Logic:** Check for a valid JWT in the Header.
      - If Token is Valid: Search Qdrant → Ask Gemini → Return Answer.
      - If Token is Missing/Invalid: Return **401 Unauthorized** error immediately.

3.  **Translation API:** `POST /api/translate`
    - **Logic:** Open for everyone (or protected, your choice). Uses **Gemini 1.5 Flash**.

4.  **Ingestion Script:** `ingest.py`
    - **Logic:** Read Markdown → FastEmbed (Local) → Upload to Qdrant.

**Technical Stack:**
- **Framework:** FastAPI (Python).
- **Security:** `python-jose` or `PyJWT` for token handling.
- **AI Model:** Google Gemini API (Free).
- **Database:** Neon Postgres (SQLAlchemy).
- **Vector DB:** Qdrant Cloud.

**Acceptance Criteria:**
- Unauthenticated requests to `/api/chat` return 401.
- Authenticated requests receive an answer from Gemini.
- Signup saves the "experience_level" correctly."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Secure Chat Interaction (Priority: P1)

A user wants to ask questions about the book content and receive answers, with their interaction being secured by authentication.

**Why this priority**: This is a core functionality that leverages both the RAG system and the new authentication, providing immediate value and demonstrating key integration points.

**Independent Test**: A user can successfully log in, obtain a JWT, and then use that JWT to make an authenticated request to the chat API, receiving a relevant answer. Unauthenticated requests to the chat API should be rejected with a 401.

**Acceptance Scenarios**:

1.  **Given** a registered user, **When** they successfully log in, **Then** they receive a valid JWT.
2.  **Given** a valid JWT, **When** a user makes a request to `POST /api/chat` with the token, **Then** they receive an answer based on book content.
3.  **Given** no valid JWT or an invalid JWT, **When** a user makes a request to `POST /api/chat`, **Then** a 401 Unauthorized error is returned.

---

### User Story 2 - User Account Management (Priority: P1)

New users can create an account with their email, password, and experience level, and existing users can log in using their credentials.

**Why this priority**: User authentication is critical for securing the chat API and managing user-specific data, making it a foundational component.

**Independent Test**: New users can sign up, and both new and existing users can log in successfully.

**Acceptance Scenarios**:

1.  **Given** unique `email`, `password`, and `experience_level`, **When** a user calls `POST /api/auth/signup`, **Then** a new user account is created in Neon Postgres with the provided details.
2.  **Given** a valid `email` and `password`, **When** a user calls `POST /api/auth/login`, **Then** they receive a JWT.
3.  **Given** an existing `email`, **When** a user attempts to sign up with the same email, **Then** an error indicating a duplicate user is returned.

---

### User Story 3 - Content Ingestion for RAG (Priority: P2)

An administrator or automated process can ingest Markdown content from the frontend documentation into the vector database (Qdrant) for the RAG system.

**Why this priority**: This story provides the data foundation for the chat API, enabling the RAG functionality. It's a backend utility that ensures the primary user story (secure chat) can function.

**Independent Test**: The ingestion script can successfully read Markdown files, process them with FastEmbed, and upload the resulting vectors and content to Qdrant.

**Acceptance Scenarios**:

1.  **Given** Markdown files in `../frontend/docs/`, **When** `ingest.py` is executed, **Then** the content is processed and uploaded to Qdrant.
2.  **Given** new or updated Markdown files, **When** `ingest.py` is executed, **Then** Qdrant is updated with the latest content.

---

### User Story 4 - Translation Service (Priority: P3)

Any user (or internal system) can send English text and receive an Urdu translation.

**Why this priority**: This is a valuable utility, but not as central to the core "Physical AI Textbook" experience as the authenticated chat.

**Independent Test**: A user can send English text to the translation API and receive a correct Urdu translation.

**Acceptance Scenarios**:

1.  **Given** English text, **When** a user calls `POST /api/translate`, **Then** an Urdu translation is returned.
2.  **Given** an empty input, **When** a user calls `POST /api/translate`, **Then** an appropriate error message is returned.

---

### Edge Cases

- What happens when an invalid JWT is provided to a protected endpoint? (Should return 401 Unauthorized)
- How does the system handle concurrent login/signup requests?
- What happens if the OpenAI/Gemini API or Qdrant Cloud services are unavailable during a request?
- How does the ingestion script handle malformed Markdown files?
- What happens if a user tries to sign up with a weak password? (Need password strength enforcement).

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide an endpoint for user signup (`POST /api/auth/signup`).
- **FR-002**: System MUST provide an endpoint for user login (`POST /api/auth/login`).
- **FR-003**: User signup MUST require `email`, `password`, and `experience_level` ("Beginner"/"Programmer").
- **FR-004**: System MUST store user data, including `email`, `password` (hashed), and `experience_level`, in Neon Postgres.
- **FR-005**: System MUST issue a JWT upon successful user login.
- **FR-006**: System MUST provide a protected chat API endpoint (`POST /api/chat`).
- **FR-007**: The protected chat API MUST validate the JWT in the request header.
- **FR-008**: If the JWT is missing or invalid for `/api/chat`, the system MUST return a 401 Unauthorized error.
- **FR-009**: The protected chat API MUST use Qdrant to search book content and Gemini to formulate answers.
- **FR-010**: System MUST provide a translation API endpoint (`POST /api/translate`).
- **FR-011**: The translation API MUST use Gemini 1.5 Flash to translate English text to Urdu.
- **FR-012**: System MUST include an ingestion script (`ingest.py`) to read Markdown files from `../frontend/docs/`.
- **FR-013**: The ingestion script MUST use FastEmbed for local embeddings and upload them to Qdrant.
- **FR-014**: API Keys (for OpenAI/Gemini, Qdrant, Neon Postgres) MUST be loaded from a `.env` file and NEVER hardcoded.
- **FR-015**: System MUST use FastAPI as the web framework.
- **FR-016**: System MUST use `python-jose` or `PyJWT` for JWT handling.
- **FR-017**: Translation API endpoint (`POST /api/translate`) protection: [NEEDS CLARIFICATION: Should the translation API be protected or open to everyone? User input states "Open for everyone (or protected, your choice)."]
- **FR-018**: Password hashing algorithm: [NEEDS CLARIFICATION: What hashing algorithm should be used for user passwords (e.g., bcrypt, scrypt)?]
- **FR-019**: JWT expiration time: [NEEDS CLARIFICATION: What should be the expiration time for issued JWTs (e.g., 1 hour, 24 hours)?]

### Key Entities

- **User**: Represents a user of the system. Key attributes include `email` (unique), `password` (hashed), and `experience_level` ("Beginner" or "Programmer").
- **BookContent**: Represents a chunk of content from the physical AI textbook, stored in Qdrant with its vector embedding.
- **JWT**: A JSON Web Token issued upon successful login, used for authenticating subsequent requests.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Unauthenticated attempts to access the chat API (`/api/chat`) consistently result in a 401 Unauthorized response.
- **SC-002**: Registered users can successfully sign up and log in, receiving a valid JWT.
- **SC-003**: User accounts created during signup correctly store the `experience_level` as either "Beginner" or "Programmer".
- **SC-004**: Authenticated requests to the chat API (`/api/chat`) receive relevant and accurate answers generated by Gemini based on the book content.
- **SC-005**: The ingestion script can successfully process and upload all Markdown files from the `frontend/docs/` directory to Qdrant.
- **SC-006**: The translation API (`/api/translate`) returns accurate Urdu translations for English input.
- **SC-007**: The backend server starts and is accessible on `localhost:8000`.