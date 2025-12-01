# Research: Physical AI & Humanoid Robotics Textbook Platform

**Date**: 2025-12-01
**Feature**: `001-ai-robotics-platform`
**Purpose**: Resolve technical unknowns and establish best practices for implementation

## Research Questions Addressed

1. **Docusaurus i18n vs Folder Routing** - How to implement static bilingual content?
2. **openai-agents SDK Integration** - How to structure Agent class with custom tools?
3. **Qdrant Chunking Strategy** - Optimal chunk size/overlap for educational content?
4. **better-auth Schema Extension** - How to add custom fields (`hardware_bg`, `software_bg`)?
5. **Neon Postgres Connection Pooling** - Best practices for serverless deployment?
6. **Vercel + Docusaurus Deployment** - Static export configuration?
7. **React Component Integration in Docusaurus** - Custom UI widgets in markdown pages?

---

## 1. Docusaurus i18n vs Folder Routing

### Decision
**Use folder-based routing** (`/docs/en/` and `/docs/ur/`) instead of Docusaurus i18n plugin.

### Rationale
- **Simplicity**: Folder routing requires zero configuration - URL structure directly maps to file structure
- **Build Performance**: i18n plugin duplicates builds for each locale; folder routing builds once
- **Translation Workflow**: Pre-generated Urdu markdown files can be validated as static assets before deployment
- **Constitution Alignment**: Principle II (Static Translation Strategy) explicitly requires URL-based switching, not runtime translation

### Alternatives Considered
- **Docusaurus i18n Plugin**: Rejected due to build overhead (2x build time for 2 languages) and unnecessary complexity for hackathon scale
- **Client-side translation**: Rejected per Constitution Principle II (violates static-only requirement)

### Implementation Approach
```javascript
// docusaurus.config.js
module.exports = {
  // NO i18n config needed
  // Content lives in:
  // - docs/en/ (English source)
  // - docs/ur/ (Urdu pre-generated)
  // LangSwitch.tsx component handles URL rewriting:
  // /docs/en/ros2/basics → /docs/ur/ros2/basics
};
```

---

## 2. openai-agents SDK Integration

### Decision
Use `openai-agents` SDK `Agent` class with a **custom `search_textbook` tool** implemented as a Python function that queries Qdrant.

### Rationale
- **Agentic Pattern**: Constitution Principle IV mandates `openai-agents` SDK for auditability and structured context
- **Tool-based RAG**: Separating vector search as a tool allows the agent to decide when to invoke it (vs. always querying)
- **Testability**: Tool functions can be unit tested independently from agent orchestration

### Alternatives Considered
- **Direct OpenAI API calls**: Rejected per Constitution Principle IV (lacks tool integration structure)
- **LangChain**: Rejected - constitution specifies `openai-agents` SDK explicitly

### Implementation Approach
```python
# api/src/agent.py
from openai_agents import Agent
from tools.search_textbook import search_textbook_tool

textbook_agent = Agent(
    name="Textbook Assistant",
    instructions="""You are a helpful tutor for the Physical AI & Humanoid Robotics course.
    Use the search_textbook tool to find relevant information from the textbook.
    Always cite which chapter/module your answer comes from.""",
    tools=[search_textbook_tool],
    model="gpt-4o-mini"  # Maps to Gemini 1.5 Flash via base_url override
)

# api/src/tools/search_textbook.py
def search_textbook_tool(query: str, selection_context: str = None) -> str:
    """Query the textbook vector database for relevant content."""
    # Use qdrant_service to perform similarity search
    # Return top-k chunks with metadata (chapter, module, language)
    pass
```

### Research Sources
- openai-agents SDK documentation: https://github.com/openai/openai-agents-sdk (conceptual pattern)
- Google Gemini via OpenAI compatibility: Requires `base_url` override in `AsyncOpenAI` client

---

## 3. Qdrant Chunking Strategy

### Decision
- **Chunk Size**: 512 tokens (~2000 characters)
- **Overlap**: 128 tokens (25% overlap)
- **Metadata**: `module`, `chapter_title`, `language` (en/ur), `source_file`

### Rationale
- **Educational Content**: Chapters contain conceptual explanations (not code-heavy docs) - 512 tokens captures a logical paragraph or concept
- **Overlap**: 25% overlap prevents context loss at chunk boundaries (e.g., multi-paragraph explanations)
- **Language-aware**: English and Urdu chunks indexed separately with `language` metadata filter for RAG queries

### Alternatives Considered
- **1024 tokens**: Rejected - too large, reduces retrieval precision (returns too much unrelated context)
- **256 tokens**: Rejected - too small, fragments concepts excessively
- **Semantic chunking** (sentence boundaries): Rejected for hackathon timeline complexity

### Implementation Approach
```python
# api/scripts/ingest.py
from qdrant_client import QdrantClient
from openai import OpenAI

def chunk_markdown(content: str, chunk_size=512, overlap=128):
    # Simple token-based chunking with overlap
    # Use tiktoken to count tokens accurately
    pass

def embed_and_ingest(chunks, metadata, collection_name="textbook"):
    client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
    openai_client = OpenAI(base_url=os.getenv("OPENAI_API_BASE"), api_key=os.getenv("GEMINI_API_KEY"))

    # Generate embeddings using Gemini via OpenAI compatibility
    embeddings = openai_client.embeddings.create(model="text-embedding-004", input=chunks)

    # Upsert to Qdrant with metadata
    client.upsert(collection_name=collection_name, points=...)
```

---

## 4. better-auth Schema Extension

### Decision
Extend default better-auth user schema with custom `hardware_bg` and `software_bg` fields using **Drizzle ORM schema extension**.

### Rationale
- **better-auth Support**: v1.x supports custom schema fields via Drizzle ORM integration
- **Type Safety**: TypeScript types auto-generated from schema for compile-time validation
- **Migration Safety**: Drizzle handles schema migrations to Neon Postgres

### Alternatives Considered
- **Separate user_profiles table**: Rejected - adds unnecessary JOIN complexity for simple fields
- **JSON field in default schema**: Rejected - loses type safety and query optimization

### Implementation Approach
```typescript
// auth/src/db/schema.ts
import { pgTable, text, timestamp, uuid } from 'drizzle-orm/pg-core';

export const users = pgTable('users', {
  id: uuid('id').primaryKey().defaultRandom(),
  email: text('email').notNull().unique(),
  password: text('password').notNull(), // hashed by better-auth
  name: text('name'),
  hardwareBg: text('hardware_bg').notNull(), // RTX4090, Jetson, Laptop, Cloud
  softwareBg: text('software_bg').notNull(), // Beginner, Intermediate, Advanced
  createdAt: timestamp('created_at').defaultNow(),
  updatedAt: timestamp('updated_at').defaultNow()
});

// auth/src/auth.config.ts
import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { users } from './db/schema';

export const auth = betterAuth({
  database: drizzleAdapter(db, { usersTable: users }),
  // ... other config
});
```

---

## 5. Neon Postgres Connection Pooling

### Decision
Use **Neon serverless driver** with built-in connection pooling for both API and Auth services.

### Rationale
- **Serverless Optimized**: Neon's HTTP-based driver designed for cold-start environments (Vercel, Render)
- **No Connection Leaks**: Automatic pooling prevents "too many connections" errors common in traditional Postgres
- **Constitution Alignment**: Principle VI (Deployment-First Design) - Neon driver works seamlessly on Render/Vercel

### Alternatives Considered
- **pg + manual pooling**: Rejected - requires warm connection pool management (doesn't fit serverless)
- **Supabase Postgres**: Rejected - Neon explicitly specified in constitution Tech Stack

### Implementation Approach
```python
# api/src/services/db_service.py
from neon_postgres import connect
import os

def get_db_connection():
    return connect(os.getenv("DATABASE_URL"))

# Queries use HTTP protocol, no persistent connections needed
```

```typescript
// auth/src/db/index.ts
import { neon } from '@neondatabase/serverless';

export const sql = neon(process.env.DATABASE_URL!);
// Drizzle ORM uses this sql client
```

---

## 6. Vercel + Docusaurus Deployment

### Decision
Use Docusaurus **static export** mode with Vercel's zero-config deployment.

### Rationale
- **Full Static**: Docusaurus builds `/web/build/` directory with static HTML/CSS/JS
- **No SSR Needed**: All content pre-generated (en/ur), no server-side rendering required
- **Edge Deployment**: Vercel serves static assets from CDN (< 500ms language switching per SC-006)

### Alternatives Considered
- **Docusaurus + Vercel Serverless Functions**: Rejected - no need for SSR, adds unnecessary complexity
- **Netlify**: Rejected - Vercel explicitly specified in constitution

### Implementation Approach
```javascript
// web/docusaurus.config.js
module.exports = {
  url: 'https://ai-robotics-textbook.vercel.app',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  staticDirectories: ['static'],
  // Build command: npm run build
  // Output: build/ directory (fully static)
};
```

```json
// vercel.json (optional - zero-config works by default)
{
  "buildCommand": "cd web && npm run build",
  "outputDirectory": "web/build",
  "framework": null
}
```

---

## 7. React Component Integration in Docusaurus

### Decision
Use **Docusaurus swizzling** to inject custom React components (PersonalizeBtn, LangSwitch, ChatWidget) into the default theme.

### Rationale
- **Native Integration**: Swizzling allows replacing/wrapping Docusaurus core components without forking
- **Persistent Widgets**: ChatWidget and LangSwitch can be added to `DocItem` wrapper (present on all doc pages)
- **State Management**: React Context API for chat widget state (no external state library needed for hackathon scale)

### Alternatives Considered
- **MDX Components**: Rejected - requires manual insertion in every markdown file
- **Browser-side injection** (script tag): Rejected - loses React hydration benefits

### Implementation Approach
```bash
# Swizzle DocItem to wrap doc pages
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap
```

```tsx
// web/src/theme/DocItem/Layout/index.tsx (swizzled)
import OriginalLayout from '@theme-original/DocItem/Layout';
import ChatWidget from '@site/src/components/ChatWidget';
import PersonalizeBtn from '@site/src/components/PersonalizeBtn';
import LangSwitch from '@site/src/components/LangSwitch';

export default function DocItemLayout(props) {
  return (
    <>
      <LangSwitch /> {/* Top-right corner */}
      <PersonalizeBtn /> {/* Below page title */}
      <OriginalLayout {...props} />
      <ChatWidget /> {/* Bottom-right floating button */}
    </>
  );
}
```

---

## 8. Testing Frameworks

### Backend (FastAPI / Python)
- **Decision**: `pytest`
- **Rationale**: `pytest` is the de-facto standard for testing in the Python ecosystem. It offers a simple, scalable, and powerful feature set, including fixture management, and has excellent integration with FastAPI. We will use it for unit, integration, and contract tests for the `/api` service.
- **Alternatives Considered**:
    - `unittest`: Built into Python but is more verbose and less flexible than `pytest`.
    - `Nose2`: While an option, it has less community momentum and a smaller plugin ecosystem compared to `pytest`.

### Frontend (Docusaurus / React)
- **Decision**: `Jest` with `React Testing Library`
- **Rationale**: `Jest` is a widely-used, full-featured testing framework for JavaScript, developed by Facebook. `React Testing Library` provides a lightweight, user-centric set of utilities for testing React components in a way that resembles how users interact with them. This combination is the standard for modern React applications and is well-supported within the Docusaurus ecosystem.
- **Alternatives Considered**:
    - `Mocha`/`Chai`: A flexible combination, but requires more configuration to set up compared to Jest's "all-in-one" approach.
    - `Cypress`/`Playwright`: These are primarily for end-to-end (E2E) testing. While they may be considered later, they are not a replacement for component-level testing with Jest and React Testing Library.

---

## Summary of Decisions

| Research Area | Decision | Key Rationale |
|---------------|----------|---------------|
| i18n Strategy | Folder routing (`/docs/en/`, `/docs/ur/`) | Simplicity, performance, constitution alignment |
| AI Agent SDK | `openai-agents` with custom `search_textbook` tool | Agentic pattern, testability per Principle IV |
| Qdrant Chunking | 512 tokens, 25% overlap, language metadata | Balances concept capture vs retrieval precision |
| better-auth Schema | Drizzle ORM extension for custom fields | Type safety, native better-auth support |
| Postgres Pooling | Neon serverless driver | Serverless-optimized, no connection leaks |
| Vercel Deployment | Docusaurus static export | Full static, edge CDN, <500ms latency |
| React Integration | Docusaurus swizzling for layout injection | Native Docusaurus pattern, persistent widgets |
| Testing (Python) | pytest | De-facto standard, FastAPI integration |
| Testing (React) | Jest + React Testing Library | Modern React standard, user-centric |

**Next Phase**: Phase 1 (Design & Contracts) - Generate data-model.md and API contracts based on these technical decisions.
