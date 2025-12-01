# Implementation Plan: Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `001-ai-robotics-platform` | **Date**: 2025-12-01 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ai-robotics-platform/spec.md`

## Summary

Build a hardware-adaptive online textbook platform for Physical AI & Humanoid Robotics that personalizes content based on users' hardware profiles (RTX4090, Jetson, Laptop, Cloud) and provides RAG-based chatbot assistance. The platform comprises three services in a monorepo: Docusaurus frontend (`/web`), FastAPI backend (`/api`), and better-auth service (`/auth`). Content is pre-generated in English and Urdu (static translation), while personalization uses LLM-driven dynamic rewriting via the `/api/personalize` endpoint. The chatbot uses `openai-agents` SDK with Google Gemini 1.5 Flash for textbook-scoped Q&A. Initial deployment targets hackathon demo scale (<100 concurrent users, ~50 chapters).

## Technical Context

**Language/Version**:
- Python 3.10+ (API service)
- Node.js 18+ / TypeScript 5+ (Auth service)
- React 18+ / TypeScript 5+ (Web service via Docusaurus 3.x)

**Primary Dependencies**:
- **API**: FastAPI 0.104+, `openai-agents` SDK, `openai` (Python client), Pydantic 2.x, `qdrant-client`, `psycopg2-binary`
- **Auth**: `better-auth` 1.x, Neon Postgres client
- **Web**: Docusaurus 3.x, React 18+, custom components for PersonalizeBtn, LangSwitch, ChatWidget

**Storage**:
- **Database**: Neon Postgres (serverless) - user profiles, session store via better-auth schema
- **Vector DB**: Qdrant Cloud - textbook content embeddings for RAG
- **Static Assets**: Generated markdown files in `/web/docs/en/` and `/web/docs/ur/`

**Testing**:
- **API**: pytest (contract tests for endpoints, integration tests for RAG agent)
- **Auth**: Jest/Vitest (unit tests for better-auth configuration)
- **Web**: Jest + React Testing Library (component tests for UI widgets)
- **Scripts**: pytest for `ingest.py` and `validate_links.py`

**Target Platform**:
- **Web**: Vercel (static export + serverless API routes)
- **API**: Render (Python 3.10+ runtime, no Docker)
- **Auth**: Render (Node.js runtime, no Docker)

**Project Type**: Web application (monorepo with 3 services)

**Performance Goals**:
- `/personalize` endpoint: <3s p95 latency (LLM rewriting)
- `/chat` endpoint: <3s p95 latency (RAG query + response)
- Language switching: <500ms (static URL navigation)
- Qdrant vector search: <200ms for textbook queries

**Constraints**:
- <100 concurrent users (hackathon demo scale)
- No Docker (Vercel/Render native deployments)
- All secrets in `.env` (never committed)
- Static Urdu translation (pre-generated, not runtime)
- Chatbot scope limited to textbook content only

**Scale/Scope**:
- ~50 Markdown chapters across 4 modules (ROS 2, Simulation, Isaac Sim, VLA)
- 4 hardware profiles (RTX4090, Jetson, Laptop, Cloud)
- Bilingual content (English primary, Urdu secondary)
- Single-user speed optimization (not high-throughput multi-tenancy)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Monorepo Architecture
✅ **PASS** - Plan explicitly structures `/web`, `/api`, `/auth` as three distinct services in monorepo.

### Principle II: Static Translation Strategy
✅ **PASS** - Content generation includes pre-built `/docs/en/` and `/docs/ur/` directories. Language switching via URL routing only (no runtime translation API).

### Principle III: Dynamic Personalization
✅ **PASS** - `/api/personalize` endpoint designed for LLM-driven hardware-specific content adaptation. User `hardware_bg` stored in Postgres and used for on-the-fly rewriting.

### Principle IV: Agentic AI Pattern
✅ **PASS** - Chatbot implementation mandates `openai-agents` SDK with `search_textbook` tool. Agent class pattern with explicit tool integration for RAG.

### Principle V: Strict Environment Configuration
✅ **PASS** - All secrets (`OPENAI_API_BASE`, `GEMINI_API_KEY`, DB credentials, Qdrant endpoints) loaded from `.env`. FR-017 requires `.env` never committed.

### Principle VI: Deployment-First Design
✅ **PASS** - Vercel (Web), Render (API/Auth) explicitly targeted. No Docker in development or production per constitution constraints.

**Gate Status**: ✅ **ALL CHECKS PASSED** - Ready for Phase 0 research.

## Project Structure

### Documentation (this feature)

```text
specs/001-ai-robotics-platform/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── api.yaml         # OpenAPI spec for /chat and /personalize endpoints
├── checklists/          # Quality validation
│   └── requirements.md  # Spec quality checklist
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Monorepo structure - Web application (3 services)

web/                        # Docusaurus frontend (React/TypeScript)
├── docs/
│   ├── en/                 # English content (Modules 1-4)
│   │   ├── ros2/           # Module 1: ROS 2 (Nervous System)
│   │   ├── simulation/     # Module 2: Simulation (Digital Twin)
│   │   ├── isaac-sim/      # Module 3: Isaac Sim (Brain)
│   │   └── vla/            # Module 4: VLA (Capstone)
│   └── ur/                 # Urdu translations (parallel structure)
│       ├── ros2/
│       ├── simulation/
│       ├── isaac-sim/
│       └── vla/
├── src/
│   ├── components/
│   │   ├── PersonalizeBtn.tsx   # Hardware-specific content toggle
│   │   ├── LangSwitch.tsx       # English/Urdu URL switcher
│   │   └── ChatWidget.tsx       # Floating RAG chatbot UI
│   ├── pages/              # Docusaurus page overrides
│   └── theme/              # Custom Docusaurus theme components
├── static/                 # Static assets (images, PDFs)
├── docusaurus.config.js    # Docusaurus configuration + i18n routing
├── package.json
├── tsconfig.json
└── .env.local              # Frontend env vars (API endpoints)

api/                        # FastAPI backend (Python 3.10+)
├── src/
│   ├── agent.py            # openai-agents Agent class implementation
│   ├── tools/
│   │   └── search_textbook.py  # RAG tool for agent
│   ├── models/
│   │   ├── chat.py         # Pydantic models for /chat endpoint
│   │   └── personalize.py  # Pydantic models for /personalize endpoint
│   ├── services/
│   │   ├── qdrant_service.py   # Vector DB client wrapper
│   │   ├── llm_service.py      # AsyncOpenAI client + personalization logic
│   │   └── db_service.py       # Neon Postgres connection
│   ├── api/
│   │   ├── chat.py         # POST /chat endpoint
│   │   └── personalize.py  # POST /personalize endpoint
│   └── main.py             # FastAPI app initialization
├── scripts/
│   ├── ingest.py           # Chunk + embed course_syllabus.md → Qdrant
│   ├── validate_links.py   # Check broken links in markdown
│   └── code_linter.py      # (Bonus) Lint code examples in docs
├── tests/
│   ├── contract/           # API contract tests
│   ├── integration/        # Agent + RAG integration tests
│   └── unit/               # Service unit tests
├── requirements.txt        # Python dependencies
├── .env.example            # Template for environment variables
└── pyproject.toml          # Python project config (optional)

auth/                       # better-auth service (Node.js/TypeScript)
├── src/
│   ├── auth.config.ts      # better-auth configuration
│   ├── db/
│   │   └── schema.ts       # User schema with hardware_bg, software_bg
│   ├── routes/
│   │   ├── signup.ts       # User registration endpoint
│   │   └── session.ts      # Session management
│   └── index.ts            # Express/Fastify server entry point
├── tests/
│   └── auth.test.ts        # Auth flow unit tests
├── package.json
├── tsconfig.json
└── .env.example            # Auth service env vars (DB, session secret)

# Root-level files
.env                        # NEVER COMMITTED - secrets for all services
.gitignore                  # Includes .env, node_modules/, __pycache__/
course_syllabus.md          # Source of truth for textbook content
README.md                   # Monorepo setup instructions
package.json                # Root workspace config (optional for monorepo tools)
```

**Structure Decision**: Web application (Option 2) selected due to distinct frontend (Docusaurus) and backend (FastAPI) with separate auth service. Three services align with Constitution Principle I (Monorepo Architecture). Each service has independent runtime/deployment target but shares `.env` configuration and `course_syllabus.md` source file.

## Complexity Tracking

> **No violations detected** - All constitutional principles satisfied by design.

No entries required in this table. The three-service architecture (`/web`, `/api`, `/auth`) is explicitly mandated by Constitution Principle I and does not exceed the approved maximum.
