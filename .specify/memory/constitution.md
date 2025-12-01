<!--
SYNC IMPACT REPORT
==================
Version Change: TEMPLATE → 1.0.0
Rationale: Initial constitution creation based on user input

Modified Principles:
- NEW: I. Monorepo Architecture
- NEW: II. Static Translation Strategy
- NEW: III. Dynamic Personalization
- NEW: IV. Agentic AI Pattern
- NEW: V. Strict Environment Configuration
- NEW: VI. Deployment-First Design

Added Sections:
- Tech Stack & Architecture Constraints
- Development Workflow Standards

Removed Sections: None (first version)

Templates Requiring Updates:
✅ .specify/templates/plan-template.md - Constitution check section is generic, compatible
✅ .specify/templates/spec-template.md - Requirements structure is compatible
✅ .specify/templates/tasks-template.md - Task organization aligns with principles
✅ CLAUDE.md - References constitution.md as source of truth, compatible

Follow-up TODOs: None - all placeholders filled
-->

# Physical AI Textbook Platform Constitution

## Core Principles

### I. Monorepo Architecture
The codebase MUST be organized as a monorepo with three distinct services:
- `/web` - Docusaurus frontend (React/TypeScript)
- `/api` - FastAPI backend (Python 3.10+)
- `/auth` - better-auth service (Node.js/TypeScript)

**Rationale**: Each service has distinct runtime requirements and deployment targets. This separation allows independent scaling and technology-appropriate implementations while maintaining unified version control and cross-service contract validation.

### II. Static Translation Strategy
Content translation MUST be static, not dynamic:
- All content generated in paired directories: `/docs/en/` and `/docs/ur/`
- UI language toggle operates via URL path switching only
- No runtime translation or LLM-based rewriting for language changes

**Rationale**: Static translation ensures consistent terminology, predictable costs, and instant page loads. Dynamic translation would introduce latency and inconsistent technical term handling in educational content.

### III. Dynamic Personalization
Hardware-specific personalization MUST be dynamic and LLM-driven:
- User's `hardware_bg` stored in Postgres upon signup
- `/api/personalize` endpoint rewrites content on-the-fly based on stored profile
- Personalization affects setup instructions AND code examples comprehensively

**Rationale**: Hardware variations (RTX4090 vs Jetson vs Laptop) require context-aware adaptation that static content cannot provide. LLM-based rewriting ensures technical accuracy while maintaining pedagogical coherence.

### IV. Agentic AI Pattern
AI features MUST use the `openai-agents` SDK with proper tool integration:
- Chatbot implemented as `Agent` class with `search_textbook` tool
- Support for `window.getSelection()` context injection
- All LLM calls routed through Google Gemini 1.5 Flash via OpenAI compatibility layer

**Rationale**: The agentic pattern with explicit tools provides auditability, testability, and controlled scope. Direct LLM calls would lack traceability and structured context handling required for educational accuracy.

### V. Strict Environment Configuration
All secrets and endpoint configurations MUST load from `.env`:
- `OPENAI_API_BASE` for Gemini endpoint
- `GEMINI_API_KEY` for authentication
- Database credentials, Qdrant endpoints
- NO hardcoded URLs or keys in source code

**Rationale**: Environment-based configuration enables secure deployment across Vercel (Web), Render (API/Auth), and local development without code changes or credential exposure.

### VI. Deployment-First Design
Architecture MUST align with target deployment platforms:
- **Web**: Vercel-compatible static export + API routes
- **API/Auth**: Render-compatible containerless deployment
- **NO Docker** in development or production

**Rationale**: Platform-native deployments reduce complexity, improve cold-start performance, and leverage platform-specific optimizations. Docker would add unnecessary abstraction for these managed platforms.

## Tech Stack & Architecture Constraints

### Backend (API Service)
- **Language**: Python 3.10+
- **Framework**: FastAPI
- **Database**: Neon Postgres (serverless)
- **Vector DB**: Qdrant Cloud
- **AI SDK**: `openai-agents` (Python)
- **LLM Provider**: Google Gemini 1.5 Flash via OpenAI compatibility

### Frontend (Web Service)
- **Framework**: Docusaurus (React/TypeScript)
- **Deployment**: Vercel
- **Build Output**: Static + API routes

### Auth Service
- **Runtime**: Node.js/TypeScript
- **Library**: better-auth
- **Database**: Shared Neon Postgres instance
- **Deployment**: Render

### Data Architecture
- **Session Store**: Postgres (better-auth schema)
- **User Profiles**: Postgres (`hardware_bg`, `software_bg`)
- **Vector Store**: Qdrant (textbook content embeddings)

## Development Workflow Standards

### API Endpoint Standards
All FastAPI endpoints MUST:
- Accept and return JSON
- Use Pydantic models for validation
- Include OpenAPI schema documentation
- Handle errors with structured responses (status code + message + detail)
- Log all requests with correlation IDs

### AI Client Initialization
AsyncOpenAI client MUST be initialized with:
```python
from openai import AsyncOpenAI
import os

client = AsyncOpenAI(
    base_url=os.getenv('OPENAI_API_BASE'),
    api_key=os.getenv('GEMINI_API_KEY')
)
```

### Content Structure
- **Textbook Source**: `course_syllabus.md` is the absolute source of truth
- **Generated Content**: Dual-language pairs in `/docs/en/` and `/docs/ur/`
- **Modules**: ROS 2, Simulation, Isaac Sim, VLA (as defined in syllabus)

### Scripts & Maintenance
- `ingest.py`: Populate Qdrant from syllabus (run on content updates)
- `validate_links.py`: Check for broken references (run pre-deployment)

## Governance

This constitution supersedes all other development practices. Any architectural decision that contradicts these principles MUST:
1. Be documented as an ADR (Architectural Decision Record)
2. Include explicit justification for the deviation
3. Propose a migration path to eventual compliance

All pull requests MUST verify:
- No hardcoded secrets or URLs
- Environment variables loaded from `.env`
- Proper separation of concerns across `/web`, `/api`, `/auth`
- Static translation + dynamic personalization boundaries respected

**Complexity Justification**: Any new service, framework, or external dependency MUST justify its necessity. The three-service architecture is the approved maximum. Additional services require documented ADR approval.

**Runtime Guidance**: Refer to `CLAUDE.md` for agent-specific development workflows and prompt engineering standards.

**Version**: 1.0.0 | **Ratified**: 2025-12-01 | **Last Amended**: 2025-12-01
