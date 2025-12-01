# Implementation Verification Checklist: Physical AI Textbook Platform

**Purpose**: Verify that implementation meets all requirements from spec.md and plan.md across monorepo structure, content generation, database schema, RAG functionality, personalization, and deployment readiness.

**Created**: 2025-12-01
**Feature**: [spec.md](../spec.md) | [plan.md](../plan.md)
**Focus Areas**: Project Structure, Content Completeness, Data Schema, AI/RAG Integration, Personalization, Deployment

---

## 1. Monorepo Structure & Configuration

- [ ] **CHK001**: Does `/web` directory exist with Docusaurus configuration (`docusaurus.config.js`, `package.json`)? [Completeness, Plan §Project Structure]
- [ ] **CHK002**: Does `/api` directory exist with FastAPI structure (`src/main.py`, `requirements.txt`)? [Completeness, Plan §Project Structure]
- [ ] **CHK003**: Does `/auth` directory exist with better-auth configuration (`src/auth.config.ts`, `package.json`)? [Completeness, Plan §Project Structure]
- [ ] **CHK004**: Is `.env` file present at repository root with all required secrets (`OPENAI_API_BASE`, `GEMINI_API_KEY`, `DATABASE_URL`, `QDRANT_URL`, `QDRANT_API_KEY`)? [Completeness, Spec §FR-017]
- [ ] **CHK005**: Is `.env` listed in `.gitignore` to prevent accidental commits? [Compliance, Spec §FR-017, Constitution Principle V]
- [ ] **CHK006**: Does root directory contain `course_syllabus.md` as the source of truth for content generation? [Completeness, Spec §FR-001]

---

## 2. Content Generation (English & Urdu)

- [ ] **CHK007**: Does `/web/docs/en/ros2/` directory exist with chapter markdown files for Module 1 (ROS 2 - Nervous System)? [Completeness, Spec §FR-002]
- [ ] **CHK008**: Does `/web/docs/en/simulation/` directory exist with chapter markdown files for Module 2 (Simulation - Digital Twin)? [Completeness, Spec §FR-002]
- [ ] **CHK009**: Does `/web/docs/en/isaac-sim/` directory exist with chapter markdown files for Module 3 (Isaac Sim - Brain)? [Completeness, Spec §FR-002]
- [ ] **CHK010**: Does `/web/docs/en/vla/` directory exist with chapter markdown files for Module 4 (VLA - Capstone)? [Completeness, Spec §FR-002]
- [ ] **CHK011**: Does `/web/docs/ur/` directory exist with parallel structure mirroring `/web/docs/en/` (ros2, simulation, isaac-sim, vla subdirectories)? [Completeness, Spec §FR-010, §FR-011]
- [ ] **CHK012**: Are Urdu markdown files pre-generated (not dynamically translated) with technical terms remaining in English? [Compliance, Spec §FR-011, §FR-012, Constitution Principle II]
- [ ] **CHK013**: Is the total chapter count approximately 50 across all 4 modules (matching scale assumption)? [Scope, Spec Clarifications §2025-12-01]

---

## 3. Database Schema & Auth

- [ ] **CHK014**: Does the `users` table schema in `/auth/src/db/schema.ts` include `hardware_bg` column/field? [Completeness, Spec §FR-003, Research §4]
- [ ] **CHK015**: Does the `users` table schema include `software_bg` column/field? [Completeness, Spec §FR-003]
- [ ] **CHK016**: Is `hardware_bg` constrained to valid values (RTX4090, Jetson, Laptop, Cloud)? [Validation, Spec §FR-003, Data Model]
- [ ] **CHK017**: Are both `hardware_bg` and `software_bg` marked as required (NOT NULL or equivalent)? [Data Integrity, Spec US1 Acceptance Scenario 2]
- [ ] **CHK018**: Is better-auth configured to use Drizzle ORM with the extended user schema? [Integration, Research §4]
- [ ] **CHK019**: Does auth service connect to Neon Postgres using the serverless driver (`@neondatabase/serverless`)? [Infrastructure, Research §5, Plan]

---

## 4. RAG & Vector Database (Qdrant)

- [ ] **CHK020**: Does `/api/scripts/ingest.py` script exist? [Completeness, Spec §FR-013]
- [ ] **CHK021**: Does `ingest.py` read `course_syllabus.md` as input for content processing? [Traceability, Spec §FR-001]
- [ ] **CHK022**: Does `ingest.py` chunk markdown content into 512-token segments with 128-token overlap? [Technical Specification, Research §3]
- [ ] **CHK023**: Does `ingest.py` generate embeddings using Google Gemini via OpenAI compatibility layer? [Integration, Research §3, Constitution]
- [ ] **CHK024**: Does `ingest.py` store embeddings in Qdrant with metadata (`module`, `chapter_title`, `language`, `source_file`)? [Data Structure, Research §3]
- [ ] **CHK025**: Can `ingest.py` be executed successfully without errors and populate Qdrant collection? [Functional Readiness]
- [ ] **CHK026**: Does Qdrant collection contain embeddings for both English and Urdu content (language-aware indexing)? [Bilingual Support, Research §3]

---

## 5. AI Agent & Chat Functionality

- [ ] **CHK027**: Does `/api/src/agent.py` implement an `Agent` class from `openai-agents` SDK? [Architecture, Spec §FR-008, Constitution Principle IV]
- [ ] **CHK028**: Does the agent include a custom `search_textbook` tool that queries Qdrant? [Tool Integration, Research §2]
- [ ] **CHK029**: Is the agent configured to use Google Gemini 1.5 Flash via `base_url` override (`AsyncOpenAI` client)? [LLM Configuration, Constitution, Research §2]
- [ ] **CHK030**: Does `POST /api/chat` endpoint accept user questions and optional `selection_context` (from `window.getSelection()`)? [API Contract, Spec §FR-009]
- [ ] **CHK031**: Does the chatbot return answers with source citations (chapter/module references)? [Response Format, Spec §FR-015, US3 Acceptance Scenario 1]
- [ ] **CHK032**: Does the chatbot reject out-of-scope questions with appropriate message (e.g., "I can only answer questions based on the textbook")? [Scope Enforcement, Spec §FR-008, US3 Acceptance Scenario 3]
- [ ] **CHK033**: Does the chat endpoint respond within 3 seconds for 95% of queries under normal load? [Performance, Spec §SC-007]

---

## 6. Personalization Feature

- [ ] **CHK034**: Does `/api/personalize` endpoint exist and accept `user_id` + `chapter_content` as input? [API Completeness, Spec §FR-005]
- [ ] **CHK035**: Does the personalization endpoint query Neon Postgres to retrieve user's `hardware_bg`? [Data Flow, Spec §FR-005]
- [ ] **CHK036**: Does the personalization logic use LLM (Gemini 1.5 Flash) to rewrite content based on `hardware_bg`? [Dynamic Rewriting, Constitution Principle III]
- [ ] **CHK037**: Does personalized content modify BOTH instructional text AND code examples? [Comprehensive Adaptation, Spec §FR-005, US2]
- [ ] **CHK038**: Does the "Personalize" button in `/web/src/components/PersonalizeBtn.tsx` toggle between personalized and original content? [UI Behavior, Spec §FR-006, US2 Acceptance Scenario 3]
- [ ] **CHK039**: Does personalization state reset when navigating to a different chapter (not persisted)? [Session Behavior, Spec US2 Acceptance Scenario 4]
- [ ] **CHK040**: Does the personalization endpoint respond within 3 seconds p95 latency? [Performance, Spec §SC-002, Plan §Performance Goals]

---

## 7. UI Components & Frontend Integration

- [ ] **CHK041**: Does `/web/src/components/PersonalizeBtn.tsx` component exist? [Component Presence, Plan §Project Structure]
- [ ] **CHK042**: Does `/web/src/components/LangSwitch.tsx` component exist for English/Urdu URL switching? [Component Presence, Spec §FR-010]
- [ ] **CHK043**: Does `/web/src/components/ChatWidget.tsx` component exist as a floating widget? [Component Presence, Spec §FR-007]
- [ ] **CHK044**: Are these components integrated into Docusaurus layout via swizzling (`/web/src/theme/DocItem/Layout/index.tsx`)? [Integration Method, Research §7]
- [ ] **CHK045**: Does `LangSwitch` component rewrite URLs from `/docs/en/*` to `/docs/ur/*` (and vice versa) without runtime translation? [URL Routing, Spec §FR-011, Constitution Principle II]
- [ ] **CHK046**: Is language switching instant (<500ms) as it uses static navigation? [Performance, Spec §SC-006]

---

## 8. Deployment Readiness

- [ ] **CHK047**: Does `/web` build successfully with `npm run build` producing static output in `/web/build/`? [Build Verification, Research §6]
- [ ] **CHK048**: Is Vercel deployment configuration present (or relying on zero-config detection)? [Deployment Config, Plan §Target Platform]
- [ ] **CHK049**: Does `/api` include `requirements.txt` with all necessary dependencies (FastAPI, openai-agents, qdrant-client, psycopg2-binary, pydantic)? [Dependency Management, Plan]
- [ ] **CHK050**: Does `/auth` include `package.json` with better-auth and Drizzle ORM dependencies? [Dependency Management, Plan]
- [ ] **CHK051**: Are API and Auth services configured for Render deployment (no Docker, native Python 3.10+/Node.js runtimes)? [Deployment Target, Constitution Principle VI]
- [ ] **CHK052**: Does the platform support <100 concurrent users without performance degradation (hackathon scale)? [Load Capacity, Spec §SC-005 (updated)]

---

## 9. Security & Validation

- [ ] **CHK053**: Do all FastAPI endpoints use Pydantic models for input validation? [Input Validation, Spec §FR-016, Constitution]
- [ ] **CHK054**: Are API error responses structured with status code + message + detail? [Error Handling, Constitution §API Endpoint Standards]
- [ ] **CHK055**: Does the signup form validate that `hardware_bg` is selected before submission? [Frontend Validation, Spec US1 Acceptance Scenario 2]
- [ ] **CHK056**: Are database credentials, API keys, and secrets loaded exclusively from `.env` (no hardcoded values)? [Security, Constitution Principle V]

---

## 10. Maintenance Scripts

- [ ] **CHK057**: Does `/api/scripts/validate_links.py` script exist? [Completeness, Spec §FR-014]
- [ ] **CHK058**: Does `validate_links.py` check for broken internal and external links in markdown files? [Functionality, Spec §FR-014]
- [ ] **CHK059**: (Bonus) Does `/api/scripts/code_linter.py` exist for linting code examples in documentation? [Bonus Feature, Plan §Project Structure]

---

## Summary Statistics

**Total Checks**: 59
**Categories**: 10 (Structure, Content, Database, RAG, Agent, Personalization, UI, Deployment, Security, Scripts)
**Focus**: Implementation verification against spec requirements and plan design

**Traceability**:
- Spec references: 35 items
- Plan references: 18 items
- Research references: 12 items
- Constitution references: 8 items

**Critical Path Items** (must pass for MVP):
- CHK001-CHK006 (Monorepo structure)
- CHK007-CHK013 (All 4 modules in both languages)
- CHK014-CHK017 (Database schema with hardware_bg)
- CHK020-CHK025 (RAG ingestion working)
- CHK027-CHK032 (Agent answers from textbook)
- CHK034-CHK040 (Personalization functional)
- CHK047-CHK051 (Deployment-ready builds)

**Notes**:
- This is an IMPLEMENTATION VERIFICATION checklist (checking if built correctly)
- NOT a requirements quality checklist (which would check if spec/plan are written correctly)
- User explicitly requested verification of built artifacts, not requirement quality assessment
- Each item checks for presence, functionality, or compliance of implemented features
