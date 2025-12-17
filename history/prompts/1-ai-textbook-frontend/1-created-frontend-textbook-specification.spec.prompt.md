----
id: 1
title: Created Frontend Textbook Specification
stage: spec
date: 2025-12-02
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-ai-textbook-frontend
branch: 1-ai-textbook-rag
user: user
command: /sp.specify Create the Frontend Specification for the "Physical AI & Humanoid Robotics" Textbook.

**Project Type:** Docusaurus (React Static Site).

**Core Structure:**
1. **Landing Page:** A modern homepage explaining the course "Physical AI & Humanoid Robotics: Bridging the gap between digital brain and physical body."
2. **Documentation (The Book):** The sidebar must strictly follow this curriculum structure:
   - **Module 1: The Robotic Nervous System (ROS 2)** (Weeks 1-5)
   - **Module 2: The Digital Twin (Gazebo & Unity)** (Weeks 6-7)
   - **Module 3: The AI-Robot Brain (NVIDIA Isaac)** (Weeks 8-10)
   - **Module 4: Vision-Language-Action (VLA)** (Weeks 11-13)

**UI/UX Requirements (Critical for Bonus Points):**
1. **Navbar Items:**
   - **"Textbook":** Link to the documentation.
   - **"Blog":** Standard Docusaurus blog section for articles/updates.
   - **"Login/Signup":** Placeholder button for Better-Auth integration.

2. **Chapter Layout (The Toolbar):** - Every chapter page must have a functional Toolbar at the top.
   - **Requirement 1:** A **"Translate to Urdu"** button. (Clicking this will eventually trigger the AI translation agent).
   - **Requirement 2:** A **"Personalize Content"** dropdown (Options: "Beginner", "Programmer").

3. **Floating Chatbot:** - A fixed "AI Assistant" button at the bottom-right for RAG-based Q&A.

**Deployment:**
- Must be configured for GitHub Pages deployment via GitHub Actions.

**Acceptance Criteria:**
- The site builds successfully.
- Navbar contains "Blog" link.
- Every chapter displays the "Translate to Urdu" button clearly.
labels: ["frontend","docusaurus","specification"]
links:
  spec: specs/1-ai-textbook-frontend/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-ai-textbook-frontend/spec.md
 - specs/1-ai-textbook-frontend/checklists/requirements.md
tests:
null
---

## Prompt

Create the Frontend Specification for the "Physical AI & Humanoid Robotics" Textbook.

**Project Type:** Docusaurus (React Static Site).

**Core Structure:**
1. **Landing Page:** A modern homepage explaining the course "Physical AI & Humanoid Robotics: Bridging the gap between digital brain and physical body."
2. **Documentation (The Book):** The sidebar must strictly follow this curriculum structure:
   - **Module 1: The Robotic Nervous System (ROS 2)** (Weeks 1-5)
   - **Module 2: The Digital Twin (Gazebo & Unity)** (Weeks 6-7)
   - **Module 3: The AI-Robot Brain (NVIDIA Isaac)** (Weeks 8-10)
   - **Module 4: Vision-Language-Action (VLA)** (Weeks 11-13)

**UI/UX Requirements (Critical for Bonus Points):**
1. **Navbar Items:**
   - **"Textbook":** Link to the documentation.
   - **"Blog":** Standard Docusaurus blog section for articles/updates.
   - **"Login/Signup":** Placeholder button for Better-Auth integration.

2. **Chapter Layout (The Toolbar):** - Every chapter page must have a functional Toolbar at the top.
   - **Requirement 1:** A **"Translate to Urdu"** button. (Clicking this will eventually trigger the AI translation agent).
   - **Requirement 2:** A **"Personalize Content"** dropdown (Options: "Beginner", "Programmer").

3. **Floating Chatbot:** - A fixed "AI Assistant" button at the bottom-right for RAG-based Q&A.

**Deployment:**
- Must be configured for GitHub Pages deployment via GitHub Actions.

**Acceptance Criteria:**
- The site builds successfully.
- Navbar contains "Blog" link.
- Every chapter displays the "Translate to Urdu" button clearly.

## Response snapshot

The frontend specification for the "Physical AI & Humanoid Robotics" Textbook has been created. The specification outlines user scenarios, functional requirements, key entities, and measurable success criteria for the Docusaurus-based static site. A quality checklist has also been generated and marked as complete.

## Outcome

- ✅ Impact: Specification for frontend textbook created.
- 🧪 Tests: No tests run.
- 📁 Files: Created specs/1-ai-textbook-frontend/spec.md and specs/1-ai-textbook-frontend/checklists/requirements.md.
- 🔁 Next prompts: Proceed to planning with `/sp.plan` or clarification with `/sp.clarify`.
- 🧠 Reflection: The `create-phr.sh` script failed, so the PHR was created using agent-native tools.

## Evaluation notes (flywheel)

- Failure modes observed: Script execution failure for `create-phr.sh`.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate script execution issues on Windows, potentially using PowerShell for PHR creation.
