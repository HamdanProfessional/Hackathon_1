# Tasks: Physical AI & Humanoid Robotics Textbook Platform

**Input**: Design documents from `specs/001-ai-robotics-platform/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/

**Tests**: Test tasks are not included as they were not explicitly requested in the specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`
- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- All file paths are relative to the repository root.

---

## Phase 1: Setup (Shared Infrastructure)
**Purpose**: High-level project initialization and directory structure.

- [ ] T001 Create project monorepo directories: `/web`, `/api`, `/auth`.
- [ ] T002 Create `.env` file from `.env.example` with placeholders for DATABASE_URL, QDRANT_URL, QDRANT_API_KEY, and GEMINI_API_KEY.
- [ ] T003 [P] Initialize a Node.js project in the `/auth` directory and add `better-auth` and `pg` as dependencies.
- [ ] T004 [P] Initialize a Python project in the `/api` directory with FastAPI and create a placeholder `main.py`.
- [ ] T005 [P] Initialize a Docusaurus project in the `/web` directory.

---

## Phase 2: Foundational (Blocking Prerequisites)
**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

- [ ] T006 Implement the basic user schema in the `/auth` service to handle user registration, login, and profile data including `hardware_bg`, based on `data-model.md`.
- [ ] T007 [P] Based on `course_syllabus.md`, generate 5 placeholder markdown chapter files in `/web/docs/en/`.
- [ ] T008 [P] Create placeholder translated markdown files for the 5 chapters in `/web/docs/ur/`.
- [ ] T009 Configure Docusaurus in `docusaurus.config.js` to support i18n with `en` and `ur` locales and a language switcher.
- [ ] T010 Implement the `scripts/ingest.py` script. It should read markdown files from `/web/docs/en`, chunk the text, and use the Qdrant client to index the embeddings.

---

## Phase 3: User Story 1 - New Student Signup (Priority: P1) 🎯 MVP
**Goal**: Allow a new user to sign up and store their hardware/software background.
**Independent Test**: A user can create an account via the UI, and the data is correctly saved by the `/auth` service.

- [ ] T011 [US1] Implement the signup endpoint in the `/auth` service that saves a new user's email, password hash, `hardware_bg`, and `software_bg`.
- [ ] T012 [US1] Create a user interface for signup in the `/web` app, including a form to collect the required information.

---

## Phase 4: User Stories 2 & 3 - Reading Experience (Priority: P2)
**Goal**: Enable content personalization and chatbot assistance for logged-in users.
**Independent Test**: A logged-in user can see personalized content and get answers from the chat widget.

- [ ] T013 [P] [US2, US3] Implement the core agent logic in `/api/src/agent.py`. The agent must include a `search_textbook` tool that queries Qdrant. **Note:** As per the constitution, initialize the `AsyncOpenAI` client with `base_url=os.getenv('OPENAI_API_BASE')` and `api_key=os.getenv('GEMINI_API_KEY')`.
- [ ] T014 [US3] Implement the `POST /chat` endpoint in `/api/src/main.py`. This endpoint should use the agent to respond to user questions.
- [ ] T015 [P] [US2] Implement the `POST /personalize` endpoint in `/api/src/main.py`. This endpoint will rewrite content based on the user's `hardware_bg`.
- [ ] T016 [P] [US2, US3] Create the React components `PersonalizeButton.tsx` and `ChatWidget.tsx` in `/web/src/components/`.
- [ ] T017 [US2, US3] Swizzle the Docusaurus `DocItem` theme component to integrate the `PersonalizeButton` and `ChatWidget` into the document layout.

---

## Phase 5: Polish & Cross-Cutting Concerns
**Purpose**: Add scripts and tools for project maintenance and quality.

- [ ] T018 [P] Implement the `scripts/validate_links.py` script to check for broken links in the markdown content.
- [ ] T019 [P] Configure and run a linter (e.g., Ruff for Python, ESLint for TypeScript/JavaScript) across the `/api`, `/auth`, and `/web` directories to ensure code quality.
- [ ] T020 [P] Create `scripts/benchmark.py` to send 10 requests to `POST /api/personalize`, calculate the p95 latency, and verify it is under 3 seconds.
- [ ] T021 [P] Setup `pytest` in the `/api` directory and write a unit test for the `POST /chat` endpoint, mocking the LLM response.

---

## Dependencies & Execution Order
- **Phase 1 (Setup)** can be done first, with T003-T005 running in parallel.
- **Phase 2 (Foundational)** must be completed after Phase 1.
- **User Story Phases (3 & 4)** can begin after Phase 2 is complete.
- **Phase 3 (Signup)** is the highest priority MVP.
- **Phase 5 (Polish)** can be worked on at any point after the relevant directories are created.
