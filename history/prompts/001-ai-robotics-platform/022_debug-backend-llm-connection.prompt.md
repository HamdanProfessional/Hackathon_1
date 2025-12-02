---
id: 022
title: debug-backend-llm-connection
stage: green
date: 2025-12-02
surface: cli
model: claude-sonnet-4-5
feature: 001-ai-robotics-platform
branch: 001-ai-robotics-platform
user: User
command: /sp.implement
labels: [debug, logging, error-handling, llm, configuration]
links:
  spec: specs/001-ai-robotics-platform/spec.md
  ticket:
  adr:
  pr:
files:
  - path: api/src/router.py
    status: modified
    lines_added: 4
    lines_removed: 2
    description: Added traceback import and detailed error logging
  - path: api/src/agents.py
    status: modified
    lines_added: 6
    lines_removed: 0
    description: Added traceback import and try/except with detailed logging
  - path: api/src/main.py
    status: modified
    lines_added: 12
    lines_removed: 0
    description: Added startup configuration debug logging
tests:
  - None
---

## Prompt

"Debug Backend LLM Connection.

1. **Update `api/agents/router.py` and `api/agents/sub_agents.py`:**
   - Locate the `try/except` blocks that currently print 'Connection error'.
   - **Action:** Replace the generic print with:
     ```python
     import traceback
     # ... inside except ...
     print(f"❌ DETAILED ERROR: {e}")
     traceback.print_exc()
     ```

2. **Verify Config:**
   - At the top of `api/main.py`, add a startup log:
     ```python
     import os
     print(f"DEBUG: Base URL: {os.getenv('OPENAI_API_BASE')}")
     print(f"DEBUG: Key Loaded: {bool(os.getenv('GEMINI_API_KEY'))}")
     ```
   - **Goal:** Reveal if the URL is wrong or the key is missing."

## Response snapshot

Added comprehensive error logging and configuration verification to debug LLM connection issues:

### 1. **Updated `api/src/router.py`** (Intent Classification Error Logging)

**Changes:**
- Line 5: Added `import traceback` for detailed stack traces
- Lines 80-81: Enhanced error logging in `classify_intent()`:
  ```python
  except Exception as e:
      print(f"❌ DETAILED ERROR in intent classification: {e}")
      traceback.print_exc()
  ```
- Lines 128-129: Enhanced error logging in `route_request()`:
  ```python
  except Exception as e:
      print(f"❌ DETAILED ERROR in agent query: {e}")
      traceback.print_exc()
  ```

**Benefits:**
- Full stack trace shows exact line where error occurs
- Clear visual indicator (❌) for errors in logs
- Identifies if error is in OpenAI client initialization, API call, or response parsing

### 2. **Updated `api/src/agents.py`** (Agent Query Error Logging)

**Changes:**
- Line 5: Added `import traceback`
- Lines 42-124: Wrapped entire `BaseAgent.query()` method in try/except:
  ```python
  try:
      # All agent logic here...
  except Exception as e:
      print(f"❌ DETAILED ERROR in BaseAgent.query(): {e}")
      traceback.print_exc()
      raise  # Re-raise to be caught by router
  ```

**Benefits:**
- Catches errors in both LLM API calls (first call with tools, second call with results)
- Shows if error is in message construction, tool execution, or response parsing
- Re-raises exception so router can handle gracefully and return error to user

### 3. **Updated `api/src/main.py`** (Startup Configuration Verification)

**Changes:**
- Line 5: Added `import os`
- Lines 13-23: Added startup debug logging:
  ```python
  print("=" * 60)
  print("🔧 API Configuration Debug")
  print("=" * 60)
  print(f"DEBUG: OPENAI_API_BASE = {os.getenv('OPENAI_API_BASE')}")
  print(f"DEBUG: GEMINI_API_KEY Loaded = {bool(os.getenv('GEMINI_API_KEY'))}")
  print(f"DEBUG: OPENAI_API_KEY Loaded = {bool(os.getenv('OPENAI_API_KEY'))}")
  print(f"DEBUG: DATABASE_URL Loaded = {bool(os.getenv('DATABASE_URL'))}")
  print(f"DEBUG: QDRANT_URL = {os.getenv('QDRANT_URL')}")
  print(f"DEBUG: QDRANT_API_KEY Loaded = {bool(os.getenv('QDRANT_API_KEY'))}")
  print("=" * 60)
  ```

**Benefits:**
- Reveals on startup if .env file is being loaded correctly
- Shows exact value of OPENAI_API_BASE (e.g., wrong URL format)
- Shows if API keys are present (without exposing actual key values)
- Helps diagnose configuration issues before any API calls are made

### Debugging Flow:

**On API Startup:**
```
============================================================
🔧 API Configuration Debug
============================================================
DEBUG: OPENAI_API_BASE = https://generativelanguage.googleapis.com/v1beta/openai/
DEBUG: GEMINI_API_KEY Loaded = True
DEBUG: OPENAI_API_KEY Loaded = True
DEBUG: DATABASE_URL Loaded = True
DEBUG: QDRANT_URL = None
DEBUG: QDRANT_API_KEY Loaded = False
============================================================
```

**On LLM Error:**
```
❌ DETAILED ERROR in intent classification: Connection error
Traceback (most recent call last):
  File "/api/src/router.py", line 59, in classify_intent
    response = await intent_client.chat.completions.create(...)
  File "/.../openai/resources/chat/completions.py", line 1234
    raise APIConnectionError(...)
openai.APIConnectionError: Connection refused: https://wrong-url.com
```

### Common Issues This Will Reveal:

1. **Missing .env**: All keys show `False`
2. **Wrong Base URL**: Shows actual URL value (e.g., missing `/v1beta/openai/`)
3. **API Key Format**: Error message will show "Invalid API key format"
4. **Network Issues**: Traceback shows connection timeout or DNS errors
5. **Model Name Wrong**: Error shows "Model not found: google/gemini-..."

## Outcome

- ✅ Impact: **Critical** - Enables debugging of LLM connection failures
- 🧪 Tests: None (debugging/logging enhancement)
- 📁 Files: 3 modified (router.py, agents.py, main.py)
- 🔁 Next prompts: Start API server and check console output for configuration, test chat endpoint to trigger error logging
- 🧠 Reflection: Detailed error logging is essential for debugging async API calls. Generic "Connection error" messages hide root causes. Stack traces + configuration verification provide complete diagnostic information.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (debugging enhancement)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add request/response logging to show exact API payloads
