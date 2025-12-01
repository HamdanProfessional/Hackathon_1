# Tasks: Physical AI & Humanoid Robotics Textbook Platform

**Input**: Design documents from `specs/001-ai-robotics-platform/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md

**Tests**: Test tasks are NOT included - not explicitly requested in specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Repository root contains three service directories:
- **web/**: Docusaurus frontend
- **api/**: FastAPI backend
- **auth/**: better-auth service

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure per user-provided setup steps

- [X] T001 Create monorepo directory structure: `web/`, `api/`, `auth/` directories at repository root
- [X] T002 Create `.env` file at repository root with provided API keys: `DATABASE_URL`, `QDRANT_URL`, `QDRANT_API_KEY`, `OPENAI_API_BASE`, `GEMINI_API_KEY`
- [X] T003 [P] Create `.gitignore` at repository root including `.env`, `node_modules/`, `__pycache__/`, `build/`, `.venv/`, `.docusaurus/`
- [X] T004 [P] Initialize Docusaurus in `web/` with `npx create-docusaurus@latest web classic --typescript` and install dependencies
- [X] T005 [P] Initialize FastAPI in `api/` with directory structure: `src/main.py`, `src/agent.py`, `src/core/config.py`
- [X] T006 [P] Create `api/requirements.txt` with dependencies: `fastapi`, `uvicorn[standard]`, `openai`, `openai-agents`, `qdrant-client`, `pydantic>=2.0`, `python-dotenv`, `psycopg2-binary`, `tiktoken`
- [ ] T007 [P] Install FastAPI dependencies: `cd api && pip install -r requirements.txt`
- [X] T008 [P] Initialize Node.js project in `auth/` with `npm init -y` and TypeScript configuration
- [ ] T009 [P] Install better-auth dependencies in `auth/`: `npm install better-auth drizzle-orm @neondatabase/serverless pg typescript @types/node`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T007 Copy `course_syllabus.md` to repository root as source of truth
- [X] T008 Create directory structure: `web/docs/en/ros2/`, `web/docs/en/simulation/`, `web/docs/en/isaac-sim/`, `web/docs/en/vla/`
- [X] T009 Create directory structure: `web/docs/ur/ros2/`, `web/docs/ur/simulation/`, `web/docs/ur/isaac-sim/`, `web/docs/ur/vla/`
- [X] T010 Generate 5 English markdown chapters from `course_syllabus.md` in `web/docs/en/` (distribute across 4 modules)
- [X] T011 Translate 5 English chapters to Urdu markdown in `web/docs/ur/` (parallel structure, technical terms in English)
- [ ] T012 Configure Docusaurus in `web/docusaurus.config.js` with folder-based routing (NOT i18n plugin)
- [ ] T013 Create `auth/src/db/schema.ts` with Drizzle ORM user schema including `hardware_bg` (RTX4090|Jetson|Laptop|Cloud) and `software_bg` fields
- [ ] T014 Create `auth/src/auth.config.ts` with better-auth configuration using Drizzle adapter
- [ ] T015 Create `api/src/services/db_service.py` for Neon Postgres connection using `@neondatabase/serverless` pattern
- [ ] T016 Create `api/scripts/ingest.py` to chunk `course_syllabus.md` (512 tokens, 128 overlap) and embed to Qdrant
- [ ] T017 Run `api/scripts/ingest.py` to populate Qdrant with English content embeddings (module, chapter_title, language=en metadata)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - New Student Signup (Priority: P1) 🎯 MVP

**Goal**: Allow new users to create accounts with hardware/software background capture

**Independent Test**: User can navigate to signup page, fill form (email, password, hardware_bg, software_bg), submit, and log in with stored profile

### Implementation for User Story 1

- [ ] T018 [US1] Create `auth/src/routes/signup.ts` implementing POST `/signup` endpoint with Pydantic validation for hardware_bg (enum constraint)
- [ ] T019 [US1] Create `web/src/pages/signup.tsx` with form collecting: name, email, password, hardware_bg dropdown (RTX4090/Jetson/Laptop/Cloud), software_bg text
- [ ] T020 [US1] Implement client-side validation in signup form: require hardware_bg selection before submission (per spec US1 acceptance scenario 2)
- [ ] T021 [US1] Create `web/src/pages/profile.tsx` to display user's stored hardware_bg after login (per spec US1 acceptance scenario 3)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Personalized Chapter Reading (Priority: P2)

**Goal**: Enable hardware-specific content adaptation via "Personalize" button

**Independent Test**: Logged-in user views chapter, clicks "Personalize," observes content changes (text + code examples) specific to their hardware_bg

### Implementation for User Story 2

- [ ] T022 [US2] Create `api/src/models/personalize.py` with Pydantic models: `PersonalizeRequest(user_id, chapter_content)`, `PersonalizeResponse(personalized_content)`
- [ ] T023 [US2] Create `api/src/services/llm_service.py` with `AsyncOpenAI` client initialized per constitution (base_url, api_key from env)
- [ ] T024 [US2] Implement LLM personalization logic in `api/src/services/llm_service.py` function: `personalize_content(content, hardware_bg)` using Gemini 1.5 Flash
- [ ] T025 [US2] Create `api/src/api/personalize.py` implementing POST `/personalize` endpoint: fetch user hardware_bg from Postgres, call llm_service
- [ ] T026 [US2] Create `web/src/components/PersonalizeBtn.tsx` React component with toggle state (personalized ↔ original)
- [ ] T027 [US2] Implement API call in `PersonalizeBtn.tsx` to POST `/personalize` with current chapter content
- [ ] T028 [US2] Ensure personalization state resets on navigation (session-based, not persisted per spec US2 acceptance scenario 4)

**Checkpoint**: At this point, User Story 2 works independently - personalization functional

---

## Phase 5: User Story 3 - Interactive Chatbot Assistance (Priority: P2)

**Goal**: RAG-based chatbot answers questions from textbook content with citations

**Independent Test**: User opens chat widget, asks textbook-related question, receives answer with chapter citation within 3 seconds

### Implementation for User Story 3

- [X] T029 [P] [US3] Create `api/src/tools/search_textbook.py` implementing `search_textbook` tool function that queries Qdrant
- [X] T030 [US3] Create `api/src/agent.py` with `openai-agents` Agent class, instructions, `search_textbook` tool, model="gpt-4o-mini" (maps to Gemini via base_url)
- [X] T031 [US3] Create `api/src/models/chat.py` with Pydantic models: `ChatRequest(question, selection_context?)`, `ChatResponse(answer, citations)`
- [X] T032 [US3] Create `api/src/api/chat.py` implementing POST `/chat` endpoint using agent, extracting citations from RAG results
- [X] T033 [US3] Implement out-of-scope rejection logic in agent instructions: respond with "I can only answer questions based on the textbook" (per spec US3 acceptance scenario 3)
- [X] T034 [P] [US3] Create `web/src/components/ChatWidget.tsx` React component as floating widget (bottom-right)
- [X] T035 [US3] Implement `window.getSelection()` context injection in `ChatWidget.tsx` (per spec FR-009)
- [X] T036 [US3] Add citation display in `ChatWidget.tsx` response rendering (link to source chapter per spec FR-015)

**Checkpoint**: At this point, User Story 3 works independently - chatbot functional with RAG + citations

---

## Phase 6: User Story 4 - Bilingual Content Access (Priority: P3)

**Goal**: URL-based language switching between English and Urdu content

**Independent Test**: User clicks language toggle, URL changes from `/docs/en/*` to `/docs/ur/*`, content displays in Urdu with technical terms in English

### Implementation for User Story 4

- [ ] T037 [P] [US4] Create `web/src/components/LangSwitch.tsx` React component detecting current URL path
- [ ] T038 [US4] Implement URL rewriting logic in `LangSwitch.tsx`: `/docs/en/` ↔ `/docs/ur/` toggle
- [ ] T039 [US4] Add session storage in `LangSwitch.tsx` to persist language preference across navigation (per spec US4 acceptance scenario 3)
- [ ] T040 [US4] Implement fallback logic: if Urdu chapter missing, display message "This chapter is currently available in English only" with link (per spec US4 acceptance scenario 4)

**Checkpoint**: At this point, User Story 4 works independently - bilingual navigation functional

---

## Phase 7: Integration & Layout

**Purpose**: Integrate all UI components into Docusaurus layout

- [X] T041 Run `npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap` in `web/` to create theme override
- [X] T042 Edit `web/src/theme/DocItem/Layout/index.tsx` to import and render `LangSwitch`, `PersonalizeBtn`, `ChatWidget` components
- [X] T043 Position components: `LangSwitch` (top-right), `PersonalizeBtn` (below page title), `ChatWidget` (floating bottom-right)

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T044 [P] Create `api/scripts/validate_links.py` to check markdown files for broken internal/external links (per spec FR-014)
- [ ] T045 [P] Create `api/scripts/code_linter.py` (bonus) to lint code examples in documentation
- [ ] T046 [P] Add `.env.example` files in `api/`, `auth/`, `web/` with placeholder environment variables
- [ ] T047 [P] Create `README.md` at repository root with monorepo setup instructions
- [ ] T048 [P] Configure CORS in `api/src/main.py` to allow requests from `web` development server
- [ ] T049 [P] Add request logging with correlation IDs in FastAPI (per constitution API Endpoint Standards)
- [ ] T050 [P] Add error handling with structured responses (status + message + detail) to all API endpoints
- [ ] T051 Run performance check: send 10 requests to `/personalize`, verify p95 < 3s (per spec SC-002)
- [ ] T052 Run performance check: send 10 requests to `/chat`, verify p95 < 3s (per spec SC-007)

---

## Phase 9: Deployment Preparation

**Purpose**: Ensure platform is ready for Vercel (web) and Render (api/auth) deployment

- [ ] T053 [P] Configure Vercel deployment: create `vercel.json` (optional, zero-config works) for `web/` static build
- [ ] T054 [P] Test Docusaurus static build: run `npm run build` in `web/`, verify output in `web/build/`
- [ ] T055 [P] Create `render.yaml` (optional) for API and Auth services deployment configuration
- [ ] T056 Verify all secrets are loaded from `.env` and no hardcoded values exist (per spec FR-017, constitution Principle V)
- [ ] T057 Generate remaining ~45 chapters from `course_syllabus.md` in English (`web/docs/en/`) to reach ~50 total
- [ ] T058 Translate remaining ~45 chapters to Urdu (`web/docs/ur/`) maintaining parallel structure
- [ ] T059 Run `api/scripts/ingest.py` on full 50-chapter corpus to populate Qdrant with complete embeddings
- [ ] T060 Test concurrent user load: simulate 100 concurrent users, verify no performance degradation (per spec SC-005 updated)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P2 → P3)
- **Integration (Phase 7)**: Depends on US2, US3, US4 component tasks (T026, T034, T037)
- **Polish (Phase 8)**: Depends on all desired user stories being complete
- **Deployment (Phase 9)**: Depends on all user stories + polish

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent of US1 (but UX assumes logged-in user)
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Independent of US1/US2 (but UX assumes logged-in user)
- **User Story 4 (P3)**: Can start after Foundational (Phase 2) - Fully independent (no auth required)

### Within Each User Story

- Models before services (e.g., T022 → T023 → T024)
- Services before endpoints (e.g., T024 → T025)
- Backend endpoints before frontend components (e.g., T025 → T026)
- Components before layout integration (e.g., T026/T034/T037 → T041-T043)

### Parallel Opportunities

- **Setup phase**: T004, T005, T006 can run in parallel
- **Foundational phase**: T013-T014 (auth schema) parallel with T015-T017 (api services)
- **After Foundation completes**: All 4 user stories (US1-US4) can start in parallel if team capacity allows
- **Within US3**: T029 (tool) parallel with T034 (widget UI)
- **Within US4**: T037 (component) parallel with other user stories
- **Polish phase**: All T044-T050 can run in parallel

---

## Parallel Example: After Foundation

```bash
# Launch all user stories in parallel (if team has 4 developers):
Developer A: Phase 3 (User Story 1 - Signup)
Developer B: Phase 4 (User Story 2 - Personalization)
Developer C: Phase 5 (User Story 3 - Chatbot)
Developer D: Phase 6 (User Story 4 - i18n)

# Each developer works through their phase's tasks sequentially
# Stories complete independently and merge without conflicts
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T017) - CRITICAL BLOCKING
3. Complete Phase 3: User Story 1 (T018-T021)
4. **STOP and VALIDATE**: Test signup flow independently
5. Deploy/demo if ready for stakeholder feedback

### Incremental Delivery (Recommended)

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (Signup) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (Personalization) → Test independently → Deploy/Demo
4. Add User Story 3 (Chatbot) → Test independently → Deploy/Demo
5. Add User Story 4 (i18n) → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers after Foundation completes:

1. Team completes Setup (Phase 1) + Foundational (Phase 2) together
2. Once Foundational done:
   - Developer A: User Story 1 (T018-T021)
   - Developer B: User Story 2 (T022-T028)
   - Developer C: User Story 3 (T029-T036)
   - Developer D: User Story 4 (T037-T040)
3. Merge in priority order: US1 → US2 → US3 → US4
4. Complete Integration (Phase 7) together
5. Divide Polish (Phase 8) tasks among team

---

## Notes

- **[P] tasks** = different files, no dependencies, safe to parallelize
- **[Story] label** maps task to specific user story for traceability
- Each user story phase is independently completable and testable
- Foundational phase (Phase 2) is the critical path - must complete before user stories
- Constitution compliance checked: monorepo structure, `.env` secrets, static translation, agentic AI, no Docker
- Performance targets: <3s p95 for `/personalize` and `/chat`, <500ms language switching
- Scale target: <100 concurrent users, ~50 chapters total
- Commit after each task or logical group for incremental progress tracking
