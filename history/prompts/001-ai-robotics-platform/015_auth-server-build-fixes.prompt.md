---
id: 015
title: auth-server-build-fixes
stage: red
date: 2025-12-02
surface: claude-code
model: claude-sonnet-4-5
feature: 001-ai-robotics-platform
branch: 001-ai-robotics-platform
user: HamdanProfessional
command: /sp.implement
labels: [debugging, build-fix, typescript, better-auth, auth-server, neon-database]
links:
  spec: specs/001-ai-robotics-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: auth/package.json
    action: verified
    reason: Confirmed scripts are correct (build, start, dev all properly configured)
  - path: auth/src/db/init.ts
    action: modified
    reason: Fixed import from 'postgres' to 'neon' and added type cast for drizzle
  - path: auth/src/index.ts
    action: modified
    reason: Fixed better-auth handler to convert Web API Request/Response to Node.js HTTP
tests:
  - Build test: npm run build (PASS - compiled successfully)
  - Dist folder: dist/index.js created successfully
---

## Prompt

Fix Auth Server Startup & Build Scripts

1. **Update `package.json` Scripts:**
   - **Build:** Ensure there is a build script: `"build": "tsc"`.
   - **Start:** Update the start script to match your source file name. If your source is `src/server.ts`, the compiled file will be `dist/server.js`.
     - Change `"start": "node dist/index.js"` to `"start": "node dist/server.js"`.
   - **Dev:** Ensure `"dev": "tsx watch src/server.ts"` exists.

2. **Compile:**
   - Run `npm run build` to generate the `dist/` folder.

## Response snapshot

Fixed TypeScript compilation errors in auth server - no package.json changes needed as scripts were already correct.

### Investigation Results

**1. Package.json Scripts ✅**
- **Status:** Already correctly configured
- **Verification:**
  ```json
  {
    "dev": "tsx watch src/index.ts",
    "build": "tsc",
    "start": "node dist/index.js"
  }
  ```
- **Entry point:** `src/index.ts` (not `src/server.ts`)
- **Compiled output:** `dist/index.js` ✓
- **Finding:** Scripts match the actual file structure perfectly

### TypeScript Compilation Errors Fixed

**Error 1: Database Import Issue**
- **File:** `auth/src/db/init.ts:10`
- **Problem:** `Module '@neondatabase/serverless' has no exported member 'postgres'`
- **Root cause:** Incorrect import - should use `neon`, not `postgres`
- **Fix:**
  ```typescript
  // Before
  import { postgres } from '@neondatabase/serverless';
  const client = postgres(process.env.DATABASE_URL);

  // After
  import { neon } from '@neondatabase/serverless';
  const client = neon(process.env.DATABASE_URL);
  ```
- **Additional fix:** Added type cast for drizzle typing issue:
  ```typescript
  const db = drizzle(client as any, { schema });
  ```

**Error 2: Better-Auth Handler API Mismatch**
- **File:** `auth/src/index.ts:65`
- **Problem:** `Expected 1 arguments, but got 2` + `IncomingMessage is not assignable to Request`
- **Root cause:** Better-Auth v1.0 uses Web API `Request`/`Response`, not Node.js HTTP types
- **Fix:** Convert between Node.js HTTP and Web API:
  ```typescript
  // Before (incorrect)
  const betterAuthHandler = await auth.handler(req, res);
  if (betterAuthHandler) {
    return;
  }

  // After (correct)
  // Convert Node.js IncomingMessage to Web API Request
  const request = new Request(`http://${req.headers.host}${req.url}`, {
    method: req.method,
    headers: req.headers as any,
  });

  const response = await auth.handler(request);
  if (response) {
    // Convert Web API Response to Node.js response
    const headers: any = {};
    response.headers.forEach((value, key) => {
      headers[key] = value;
    });
    res.writeHead(response.status, headers);
    res.end(await response.text());
    return;
  }
  ```

### Build Verification

✅ **Compilation:** Successful
```
> tsc
(no errors)
```

✅ **Output Files Generated:**
```
dist/
├── auth.config.js
├── auth.config.d.ts
├── index.js            # ✓ Main entry point
├── index.d.ts
├── db/
│   ├── init.js
│   └── schema.js
└── routes/
```

### Root Cause Analysis

**Issue:** TypeScript errors preventing compilation, not npm script configuration

1. **@neondatabase/serverless API:** Exports `neon` function, not `postgres` class
2. **Better-Auth v1.0 API:** Uses standard Web API Request/Response instead of Node.js http module types
3. **Drizzle typing:** Strict type checking required type assertion for neon client

### Scripts Status

**All package.json scripts are correctly configured:**
- ✅ `"build": "tsc"` - Compiles TypeScript to dist/
- ✅ `"start": "node dist/index.js"` - Runs compiled server (correct path)
- ✅ `"dev": "tsx watch src/index.ts"` - Development mode with hot reload

**No changes needed** - the original scripts were already properly aligned with the file structure.

## Outcome

- ✅ Impact: Auth server can now compile successfully and dist/ folder is generated with index.js entry point for production use
- 🧪 Tests: Build test PASS, output verification PASS
- 📁 Files: 2 source files modified (db/init.ts, index.ts), package.json verified correct
- 🔁 Next prompts: Test server startup with `npm start`, verify database connection, test auth endpoints
- 🧠 Reflection: Better-Auth v1.0 migration to Web API standards (Request/Response) requires adapter code when using Node.js http module. The conversion between APIs is straightforward but essential.

## Evaluation notes (flywheel)

- Failure modes observed: TypeScript compilation failed due to API mismatch (better-auth expects Web API Request, not Node IncomingMessage) and incorrect Neon import (used 'postgres' instead of 'neon')
- Graders run and results (PASS/FAIL): TypeScript compilation PASS after fixes, dist folder generation PASS
- Prompt variant (if applicable): Debugging/fixing prompt (stage: red)
- Next experiment (smallest change to try): Consider using a better-auth adapter package if available, or create a reusable middleware function for Web API ↔ Node.js HTTP conversion
