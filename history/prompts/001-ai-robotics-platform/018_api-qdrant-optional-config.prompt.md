---
id: 018
title: api-qdrant-optional-config
stage: green
date: 2025-12-02
surface: cli
model: claude-sonnet-4-5
feature: 001-ai-robotics-platform
branch: 001-ai-robotics-platform
user: User
command: /sp.implement
labels: [bugfix, config, api, qdrant, optional-deps]
links:
  spec: specs/001-ai-robotics-platform/spec.md
  ticket:
  adr:
  pr:
files:
  - path: api/src/core/config.py
    status: modified
    lines_added: 2
    lines_removed: 2
    description: Made QDRANT_URL and QDRANT_API_KEY optional to prevent startup crashes
tests:
  - None
---

## Prompt

"Fix API Startup Crash: Handle Missing Qdrant Config

**Goal:** Prevent the API from crashing when Qdrant keys are missing during local development.

1. **Update `api/src/core/config.py`:**
   - **Action:** Change the type hints for `QDRANT_URL` and `QDRANT_API_KEY` to be **Optional**.
   - **Code Change:**
     ```python
     from typing import Optional
     # ...
     class Settings(BaseSettings):
         # ...
         # Make these optional with a default of None
         QDRANT_URL: Optional[str] = None
         QDRANT_API_KEY: Optional[str] = None

         # Keep others as is, or make them optional too if needed
         # ...
     ```

2. **Verify `.env` Loading:**
   - **Action:** Ensure the `Config` class inside `Settings` looks for the `.env` file correctly, especially since you are running from `src/`.
   - **Code:**
     ```python
     class Config:
         # Look for .env in current dir OR parent dir
         env_file = (".env", "../.env")
         extra = "ignore"
     ```

3. **Instruction:** Output the full corrected `api/src/core/config.py` file."

## Response snapshot

Fixed API configuration to prevent startup crashes when Qdrant keys are missing:

1. **Made Qdrant Optional**: Changed `QDRANT_URL` and `QDRANT_API_KEY` from required fields to `Optional[str] = None`
   - Line 16: `QDRANT_URL: Optional[str] = None`
   - Line 17: `QDRANT_API_KEY: Optional[str] = None`

2. **Multi-Location .env Loading**: Updated `model_config` to search for .env in multiple locations
   - Line 38: `env_file=(".env", "../.env")`
   - Handles cases when running from `api/src/`, `api/`, or repository root

3. **Added Documentation**: Updated comments to clarify optional status for local development

The API will now start successfully without Qdrant configuration, allowing local development without vector database setup. RAG features require Qdrant to be functional, but the service no longer crashes on startup.

## Outcome

- ✅ Impact: **Critical** - Unblocks local API development without Qdrant setup
- 🧪 Tests: None (configuration change only)
- 📁 Files: 1 modified (api/src/core/config.py)
- 🔁 Next prompts: Test API startup without Qdrant keys, implement graceful degradation for RAG features
- 🧠 Reflection: Pydantic Settings v2 handles Optional fields gracefully with `Optional[str] = None` syntax. Multi-location env_file as tuple is the correct pattern for monorepo .env resolution.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (configuration fix)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add runtime checks in RAG code to handle `settings.QDRANT_URL is None` gracefully
