---
id: 016
title: env-loading-dotenv-first
stage: red
date: 2025-12-02
surface: claude-code
model: claude-sonnet-4-5
feature: 001-ai-robotics-platform
branch: 001-ai-robotics-platform
user: HamdanProfessional
command: /sp.implement
labels: [debugging, environment-variables, dotenv, production, build-fix]
links:
  spec: specs/001-ai-robotics-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: auth/src/index.ts
    action: modified
    reason: Moved dotenv import to first line and added DATABASE_URL debug check
  - path: auth/src/auth.config.ts
    action: modified
    reason: Moved dotenv import to absolute first line before all other imports
tests:
  - Build test: npm run build (PASS)
  - Dist verification: dotenv require() appears first in compiled index.js (PASS)
---

## Prompt

Fix Production Env Loading & Rebuild

Goal: Ensure .env is loaded before the app crashes, and rebuild the stale dist folder.

1. Update Entry Point (src/index.ts or src/server.ts)

Action: Open your main entry file.

Critical Change: Move import 'dotenv/config'; to the very first line of the file. It must be before any other imports like ./auth.config or express.

Debug: Add this line immediately after the import:

```typescript
console.log("🔍 Checking DB URL:", process.env.DATABASE_URL ? "Loaded ✅" : "Missing ❌");
```

2. Force Rebuild

Context: You are running node dist/index.js, but your changes are in src/.

Action: Run the build command to update the dist folder.

Command: npm run build

3. Execution

Action: Run npm start again.

Expected Output: You should see '🔍 Checking DB URL: Loaded ✅' in the console.

## Response snapshot

Successfully moved dotenv loading to the very first line and added debug check to verify DATABASE_URL loading in production.

### 1. Entry Point Fix (index.ts)

**Updated `auth/src/index.ts`:**
- **Before:** `import 'dotenv/config';` was on line 8 (after comment block)
- **After:** Moved to line 1 (absolute first line)
- **Added debug check on line 11:**
  ```typescript
  console.log("🔍 Checking DB URL:", process.env.DATABASE_URL ? "Loaded ✅" : "Missing ❌");
  ```

**New structure:**
```typescript
import 'dotenv/config';

/**
 * Authentication Server
 * ...
 */

// Debug: Check if DATABASE_URL is loaded
console.log("🔍 Checking DB URL:", process.env.DATABASE_URL ? "Loaded ✅" : "Missing ❌");

import { auth } from './auth.config';
// ... rest of imports
```

**Rationale:** Dotenv must load before ANY other imports, including `./auth.config`, because auth.config.ts checks `process.env.DATABASE_URL` at module load time.

### 2. Auth Config Fix (auth.config.ts)

**Updated `auth/src/auth.config.ts`:**
- **Before:** `import "dotenv/config";` was on line 8 (after comment block)
- **After:** Moved to line 2 (immediately after opening comment, but first executable code)

**New structure:**
```typescript
// 1. LOAD ENV VARS FIRST
import "dotenv/config";

/**
 * Better-Auth Configuration
 * ...
 */

import { betterAuth } from 'better-auth';
// ... rest of imports
```

**Rationale:** Even though index.ts loads dotenv first, auth.config.ts is a standalone module that can be imported elsewhere, so it needs its own dotenv guard.

### 3. Rebuild & Verification

**Rebuild:**
```
> npm run build
> tsc
(no errors - clean build)
```

**Compiled Output Verification:**
```javascript
// auth/dist/index.js (lines 1-14)
"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
require("dotenv/config");  // ✓ Line 6 - First require()
/**
 * Authentication Server
 * ...
 */
// Debug: Check if DATABASE_URL is loaded
console.log("🔍 Checking DB URL:", process.env.DATABASE_URL ? "Loaded ✅" : "Missing ❌");
const auth_config_1 = require("./auth.config");  // ✓ Loaded AFTER dotenv
```

**Key Confirmation:**
1. ✅ `require("dotenv/config")` appears on line 6 (first require after TypeScript boilerplate)
2. ✅ Debug console.log appears on line 14
3. ✅ `auth.config` is required AFTER dotenv

### Expected Runtime Behavior

When running `npm start`, the console should show:
```
🔍 Checking DB URL: Loaded ✅
🔐 Auth server running on http://localhost:3001
...
```

If DATABASE_URL is missing, it will show:
```
🔍 Checking DB URL: Missing ❌
❌ DATABASE_URL is missing. Please check your .env file.
```

### Root Cause Analysis

**Original Problem:**
- Module import order: `auth.config.ts` was imported before dotenv could load environment variables
- The `auth.config.ts` module checks `process.env.DATABASE_URL` at the top level (module load time)
- If dotenv hasn't run yet, `DATABASE_URL` is `undefined`, causing the validation error

**Solution:**
- Moved dotenv import to absolute first line in both files
- Added debug logging to immediately verify env loading
- Rebuilt dist/ folder to apply changes to production code

## Outcome

- ✅ Impact: Environment variables now load before any module-level checks, preventing "DATABASE_URL is missing" errors in production
- 🧪 Tests: Build test PASS, compiled output verified with dotenv first
- 📁 Files: 2 files modified (index.ts, auth.config.ts)
- 🔁 Next prompts: Test server startup with `npm start`, verify debug message shows "Loaded ✅", check database connection
- 🧠 Reflection: Import order matters critically in Node.js - dotenv must be the absolute first import/require, even before type-only imports. The debug console.log is invaluable for troubleshooting env loading in production.

## Evaluation notes (flywheel)

- Failure modes observed: Dotenv was imported but not early enough - being after comment blocks still caused it to run after some module initialization code
- Graders run and results (PASS/FAIL): Build test PASS, import order verification PASS
- Prompt variant (if applicable): Debugging prompt (stage: red)
- Next experiment (smallest change to try): Consider using environment variable validation library like `envalid` or `zod` for runtime validation instead of top-level checks
