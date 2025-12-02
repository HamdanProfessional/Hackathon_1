---
id: 019
title: auth-cors-credentials-fix
stage: green
date: 2025-12-02
surface: cli
model: claude-sonnet-4-5
feature: 001-ai-robotics-platform
branch: 001-ai-robotics-platform
user: User
command: /sp.implement
labels: [bugfix, cors, auth, credentials, security]
links:
  spec: specs/001-ai-robotics-platform/spec.md
  ticket:
  adr:
  pr:
files:
  - path: auth/src/index.ts
    status: modified
    lines_added: 7
    lines_removed: 3
    description: Fixed CORS configuration to allow credentials with explicit origins
tests:
  - None
---

## Prompt

"Fix CORS Credentials Error in Auth Server

**Goal:** Allow the Frontend (localhost:3000) to send credentials (cookies) to the Auth Server (localhost:3001).

**Problem:** The server is returning `Access-Control-Allow-Origin: *`, which is forbidden when `credentials: include` is used.

**Action:** Update `auth/src/index.ts` (or `server.ts`).

1.  **Locate CORS Middleware:** Find where `app.use(cors(...))` is called.
2.  **Update Configuration:** Change it to explicitly allow the frontend origin.
    ```typescript
    app.use(cors({
        origin: ["http://localhost:3000", "http://localhost:8000"], // Must be explicit strings, NOT '*'
        credentials: true, // Required for cookies
        methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
        allowedHeaders: ["Content-Type", "Authorization"]
    }));
    ```
3.  **Placement:** Ensure this middleware is defined **before** your routes (`app.all(...)`)."

## Response snapshot

Fixed CORS credentials error in auth server by replacing wildcard origin with explicit origin validation:

**Changes Made to `auth/src/index.ts` (lines 30-47)**:

1. **Removed Wildcard Origin** (line 32):
   - **Before**: `res.setHeader('Access-Control-Allow-Origin', '*');`
   - **After**: Dynamic origin validation based on request origin

2. **Added Explicit Origin Validation** (lines 32-37):
   ```typescript
   const allowedOrigins = ['http://localhost:3000', 'http://localhost:8000'];
   const origin = req.headers.origin || '';

   if (allowedOrigins.includes(origin)) {
     res.setHeader('Access-Control-Allow-Origin', origin);
   }
   ```

3. **Added Credentials Header** (line 39):
   - Added: `res.setHeader('Access-Control-Allow-Credentials', 'true');`

4. **Maintained Other CORS Headers**:
   - `Access-Control-Allow-Methods`: GET, POST, PUT, DELETE, OPTIONS
   - `Access-Control-Allow-Headers`: Content-Type, Authorization

**How It Works**:
- Checks if incoming request origin is in `allowedOrigins` array
- Only sets `Access-Control-Allow-Origin` header for allowed origins
- Always sets `Access-Control-Allow-Credentials: true` to enable cookie transmission
- Properly handles OPTIONS preflight requests

**Security Benefits**:
- Prevents CORS bypass attacks by restricting origins
- Enables secure session cookie transmission
- Maintains browser same-origin security policies

**Rebuild**:
- Ran `npm run build` successfully
- Verified compiled `dist/index.js` contains correct CORS logic

## Outcome

- ✅ Impact: **Critical** - Enables frontend authentication with session cookies
- 🧪 Tests: None (CORS configuration fix)
- 📁 Files: 1 modified (auth/src/index.ts)
- 🔁 Next prompts: Test authentication flow from frontend, verify cookie persistence
- 🧠 Reflection: CORS with credentials requires explicit origin matching - wildcard `*` is forbidden by CORS spec. Dynamic origin validation from request headers is the correct pattern for multi-origin support.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (configuration fix)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add environment variable for allowed origins to support production domains
