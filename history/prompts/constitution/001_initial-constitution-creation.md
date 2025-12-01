---
id: 001
title: initial-constitution-creation
stage: constitution
date: 2025-12-01
surface: claude-code
model: claude-sonnet-4-5
feature: N/A
branch: master
user: Hamdan
command: /sp.constitution
labels: [constitution, initialization, v1.0.0]
links:
  spec: N/A
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: .specify/memory/constitution.md
    status: created
    lines: 158
tests:
  - N/A
---

## Prompt

/sp.constitution "1. **Tech Stack & Architecture:**
   - **Repository:** Monorepo (`/web`, `/api`, `/auth`).
   - **Frontend:** Docusaurus (React/TS).
   - **Backend:** FastAPI (Python 3.10+).
   - **Auth:** `better-auth` (Node.js/TypeScript) + Neon Postgres.
   - **Vector DB:** Qdrant Cloud.
   - **AI Engine:** `openai-agents` SDK (Python) using Google Gemini 1.5 Flash via OpenAI Compatibility.

2. **Core Functionality Rules:**
   - **Translation:** **Static**. Content must be generated in pairs: `/docs/en/` and `/docs/ur/`. The UI button simply toggles the URL path.
   - **Personalization:** **Dynamic**. The `/api/personalize` endpoint rewrites text on-the-fly based on the user's `hardware_bg` stored in Postgres.
   - **Chatbot:** **Agentic**. Must use `openai-agents` `Agent` class with a `search_textbook` tool. It must support `window.getSelection()` context.

3. **Configuration:**
   - **Secrets:** Load strictly from `.env`.
   - **LLM Client:** Initialize `AsyncOpenAI` with `base_url=os.getenv('OPENAI_API_BASE')` and `api_key=os.getenv('GEMINI_API_KEY')` with Openai_Agents sdk.
   - **No Docker:** Deployment targets are Vercel (Web) and Render (API/Auth)."

## Response snapshot

Created the initial project constitution (v1.0.0) for the Physical AI & Humanoid Robotics Textbook Platform.

**Constitution Structure:**
- 6 Core Principles: Monorepo Architecture, Static Translation Strategy, Dynamic Personalization, Agentic AI Pattern, Strict Environment Configuration, Deployment-First Design
- Tech Stack & Architecture Constraints section defining all technology choices
- Development Workflow Standards with concrete implementation patterns
- Governance rules for compliance and ADR requirements

**Key Principles Encoded:**
1. Mandated three-service monorepo structure (/web, /api, /auth)
2. Static translation (URL-based) vs dynamic personalization (LLM-based) clearly separated
3. Agentic AI pattern using openai-agents SDK as mandatory approach
4. Environment-based configuration (.env) for all secrets and endpoints
5. No Docker, platform-native deployments (Vercel/Render)

**Template Validation:**
- ✅ plan-template.md: Constitution check section is generic placeholder, compatible
- ✅ spec-template.md: Requirements structure aligns with principles
- ✅ tasks-template.md: Task organization supports monorepo and principle-driven development
- ✅ CLAUDE.md: References constitution as source of truth, no conflicts

**Version Rationale:**
- 1.0.0 chosen as initial constitution
- Ratification date: 2025-12-01
- No prior version history

## Outcome

- ✅ Impact: Established authoritative governance document for the entire project lifecycle
- 🧪 Tests: N/A (governance document)
- 📁 Files: Created .specify/memory/constitution.md (158 lines)
- 🔁 Next prompts: /sp.specify, /sp.plan, or /sp.tasks can now reference constitution principles
- 🧠 Reflection: Constitution successfully encodes all user requirements as testable, enforceable rules. The separation of static translation vs dynamic personalization is clearly articulated to prevent implementation confusion.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Monitor adherence during /sp.plan execution - verify constitution check gates are properly applied
