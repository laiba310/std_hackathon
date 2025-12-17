# Tasks: Frontend for "Physical AI & Humanoid Robotics" Textbook

**Input**: Design documents from `/specs/1-ai-textbook-frontend/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The acceptance criteria from the spec will serve as the basis for testing each task.

**Organization**: Tasks are grouped by the user's defined phases and structured for testability and manageable scope.

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions

---

## Phase 1: Setup & Theming (Task 1)

**Purpose**: Initialize Docusaurus and implement the custom Orange/Black (Dark) and Orange/White (Light) theme.

- [ ] T001 Initialize Docusaurus project in the repository root (if not already initialized).
- [ ] T002 [P] Create or modify `src/css/custom.css` to define the Orange/Black (Dark) and Orange/White (Light) themes, overriding Docusaurus defaults.
- [ ] T003 [P] Configure `docusaurus.config.js` to correctly load the custom CSS and ensure theme modes are applied.
- [ ] T004 Verify the Docusaurus site builds successfully with the custom theme applied.
- [ ] T005 Verify the custom Orange theme meets WCAG 2.1 AA contrast ratios for accessibility.

---

## Phase 2: Structure (Task 2)

**Purpose**: Create the `sidebar.js` logic and empty folder structure for Modules 1-4.

- [ ] T006 Create or modify `sidebars.js` to define the curriculum structure for Modules 1-4:
    - Module 1: The Robotic Nervous System (ROS 2)
    - Module 2: The Digital Twin (Gazebo & Unity)
    - Module 3: The AI-Robot Brain (NVIDIA Isaac)
    - Module 4: Vision-Language-Action (VLA)
- [ ] T007 [P] Create empty directory structures for `docs/module1`, `docs/module2`, `docs/module3`, `docs/module4` to house the content.
- [ ] T008 Verify the sidebar structure is correctly displayed in the Docusaurus navigation.

---

## Phase 3: Content Generation (Tasks 3, 4, 5)

**Purpose**: Generate Markdown content for all Modules.

### Task 3: Content - Module 1 (Robotic Nervous System)

- [ ] T009 Generate Markdown content for Module 1 (Robotic Nervous System) and save it to `docs/module1/`.
- [ ] T010 Verify Module 1 content is accessible and displays correctly through the sidebar navigation.

### Task 4: Content - Module 2 (Digital Twin)

- [ ] T011 Generate Markdown content for Module 2 (Digital Twin) and save it to `docs/module2/`.
- [ ] T012 Verify Module 2 content is accessible and displays correctly through the sidebar navigation.

### Task 5: Content - Modules 3 & 4 (AI-Robot Brain, Vision-Language-Action)

- [ ] T013 Generate Markdown content for Module 3 (AI-Robot Brain) and save it to `docs/module3/`.
- [ ] T014 Generate Markdown content for Module 4 (Vision-Language-Action) and save it to `docs/module4/`.
- [ ] T015 Verify Module 3 content is accessible and displays correctly through the sidebar navigation.
- [ ] T016 Verify Module 4 content is accessible and displays correctly through the sidebar navigation.

---

## Phase 4: UI Components (Task 6)

**Purpose**: Build the "Chapter Toolbar" (Urdu/Personalize buttons) and "Floating Chatbot".

- [ ] T017 Create a React component for the "Chapter Toolbar" including a "Translate to Urdu" button and a "Personalize Content" dropdown (`src/components/ChapterToolbar/index.js`).
- [ ] T018 Integrate the "Chapter Toolbar" component into Docusaurus chapter pages (e.g., via `src/theme/DocItem/Content/index.js` override).
- [ ] T019 Verify the "Translate to Urdu" button is visible on every chapter page.
- [ ] T020 Verify the "Personalize Content" dropdown is visible on every chapter page with "Beginner" and "Programmer" options.
- [ ] T021 Create a React component for the "Floating Chatbot" button and its pop-up chat interface (`src/components/FloatingChatbot/index.js`).
- [ ] T022 Integrate the "Floating Chatbot" component globally into the Docusaurus layout (e.g., via `src/theme/Layout/index.js` override).
- [ ] T023 Verify the "AI Assistant" button is fixed at the bottom-right and accessible on all pages.

---

## Phase 5: Logic (Task 7 - Dummy API Calls & Auth Placeholder)

**Purpose**: Connect dummy API calls for personalization and translation, and implement the Auth placeholder.

- [ ] T024 Implement dummy API calls for the "Translate to Urdu" button, simulating a backend proxy response.
- [ ] T025 Implement dummy API calls for the "Personalize Content" dropdown, simulating fetching/storing user preferences from a backend database.
- [ ] T026 Update `docusaurus.config.js` to include the "Login/Signup" placeholder button in the navbar, integrating directly with a dummy backend API endpoint.
- [ ] T027 Verify the "Login/Signup" button is present in the navbar and shows a placeholder interaction on click.

---

## Phase 6: Deployment Configuration (Task 8)

**Purpose**: Configure GitHub Pages deployment via GitHub Actions.

- [ ] T028 Create or modify `.github/workflows/deploy.yml` to configure GitHub Actions for Docusaurus deployment to GitHub Pages.
- [ ] T029 Verify the GitHub Actions workflow successfully builds and deploys the site to GitHub Pages.
