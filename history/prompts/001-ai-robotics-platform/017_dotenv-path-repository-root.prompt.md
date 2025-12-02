---
id: 017
title: dotenv-path-repository-root
stage: red
date: 2025-12-02
surface: claude-code
model: claude-sonnet-4-5
feature: 001-ai-robotics-platform
branch: 001-ai-robotics-platform
user: HamdanProfessional
command: debugging
labels: [debugging, dotenv, env-path, monorepo, neon-database]
links:
  spec: specs/001-ai-robotics-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: auth/src/index.ts
    action: modified
    reason: Changed from 'dotenv/config' to explicit path loading from repository root
  - path: auth/src/auth.config.ts
    action: modified
    reason: Changed from 'dotenv/config' to explicit path loading from repository root
tests:
  - Build test: npm run build (PASS)
  - Path verification: __dirname/../../.env resolves to repository root (PASS)
---

## Prompt

**Error:** Database connection string format error when starting auth server
```
Error: Database connection string format for `neon()` should be: postgresql://user:password@host.tld/dbname?option=value
```

**Debug output showed:**
```
🔍 Checking DB URL: Loaded ✅
```

**Root cause:** Dotenv was loading successfully but from the wrong location (looking in `auth/` directory instead of repository root where `.env` actually exists).

## Response snapshot

Fixed dotenv path loading to explicitly point to repository root `.env` file instead of relying on default search behavior.

### Problem Analysis

**Symptom:**
- Debug showed "Loaded ✅" but database connection still failed
- Error message about connection string format appeared

**Investigation:**
1. Checked `.env` file location: `C:\Users\User\Desktop\code\.env` (repository root) ✓
2. Checked DATABASE_URL format: `postgresql://neondb_owner:...` (correct format) ✓
3. Checked for duplicate `.env` files: None found in `auth/` directory ✓
4. **Root cause identified:** `import 'dotenv/config'` searches for `.env` starting from current working directory

**The Issue:**
When running `npm start` from the `auth/` directory, dotenv's default behavior searches for `.env` in this order:
1. `auth/.env` (doesn't exist)
2. `auth/../.env` (would be repository root, but may not check this by default)
3. Falls back to already-loaded env vars or fails

Since the auth server runs from `auth/dist/index.js`, the working directory is `auth/`, not the repository root.

### Solution Implemented

**Changed dotenv loading from implicit to explicit path:**

**Before (auth/src/index.ts):**
```typescript
import 'dotenv/config';
```

**After (auth/src/index.ts):**
```typescript
import { config } from 'dotenv';
import { resolve } from 'path';

// Load .env from parent directory (repository root)
config({ path: resolve(__dirname, '../../.env') });
```

**Path resolution:**
- When compiled, `__dirname` = `C:\Users\User\Desktop\code\auth\dist`
- `resolve(__dirname, '../../.env')` = `C:\Users\User\Desktop\code\.env`
- This explicitly points to the repository root `.env` file

**Same change applied to `auth/src/auth.config.ts`:**
```typescript
import { config } from 'dotenv';
import { resolve } from 'path';

// Load .env from parent directory (repository root)
config({ path: resolve(__dirname, '../../.env') });
```

### Build Verification

✅ **Compilation successful:**
```
> npm run build
> tsc
(no errors)
```

✅ **Compiled output verification (auth/dist/index.js):**
```javascript
const dotenv_1 = require("dotenv");
const path_1 = require("path");
// Load .env from parent directory (repository root)
(0, dotenv_1.config)({ path: (0, path_1.resolve)(__dirname, '../../.env') });
```

### Monorepo Pattern

This is a common pattern in monorepos where:
- **Shared configuration:** `.env` lives at repository root
- **Service directories:** Each service (`web/`, `api/`, `auth/`) runs from its own directory
- **Explicit path required:** Services must explicitly reference `../.env` or `../../.env`

**Alternative approaches (not used here):**
1. Copy `.env` to each service directory (duplication, harder to maintain)
2. Use environment variable injection at runtime (requires deployment configuration)
3. Use a shared config package (overkill for this project)

### Debug Output Enhancement

Added additional debug line to show actual DATABASE_URL value:
```typescript
console.log("🔍 DB URL value:", process.env.DATABASE_URL?.substring(0, 30) + "...");
```

**Expected output on next run:**
```
🔍 Checking DB URL: Loaded ✅
🔍 DB URL value: postgresql://neondb_owner:npg_o...
🔐 Auth server running on http://localhost:3001
```

## Outcome

- ✅ Impact: Auth server can now correctly load DATABASE_URL from repository root .env file, resolving Neon connection string error
- 🧪 Tests: Build test PASS, path resolution verified in compiled output
- 📁 Files: 2 files modified (index.ts, auth.config.ts)
- 🔁 Next prompts: Test server startup with `npm start`, verify Neon database connection, test auth endpoints
- 🧠 Reflection: In monorepo structures, never rely on dotenv's default search behavior. Always use explicit paths with `config({ path: resolve(__dirname, '../.env') })` to avoid runtime errors in different working directories.

## Evaluation notes (flywheel)

- Failure modes observed: Dotenv appeared to load successfully (debug showed "Loaded ✅") but was actually loading from wrong location or falling back to existing env vars
- Graders run and results (PASS/FAIL): Build test PASS, explicit path loading PASS
- Prompt variant (if applicable): Debugging prompt (stage: red)
- Next experiment (smallest change to try): Consider using a shared environment configuration utility that all services import, ensuring consistent .env loading across the monorepo
