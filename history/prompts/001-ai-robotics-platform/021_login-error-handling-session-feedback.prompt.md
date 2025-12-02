---
id: 021
title: login-error-handling-session-feedback
stage: green
date: 2025-12-02
surface: cli
model: claude-sonnet-4-5
feature: 001-ai-robotics-platform
branch: 001-ai-robotics-platform
user: User
command: /sp.implement
labels: [bugfix, auth, ux, error-handling, session]
links:
  spec: specs/001-ai-robotics-platform/spec.md
  ticket:
  adr:
  pr:
files:
  - path: web/src/pages/login.tsx
    status: modified
    lines_added: 15
    lines_removed: 8
    description: Fixed login to check response.error before redirecting
  - path: web/src/components/AuthNavbarItem.tsx
    status: modified
    lines_added: 15
    lines_removed: 4
    description: Added session debugging and page reload on logout
tests:
  - None
---

## Prompt

"Fix Login Error Handling and Session Feedback
**Goal:** Stop the Login page from redirecting blindly. It must only redirect on success and display errors if authentication fails. Additionally, ensure the Navbar reflects the active session immediately.

1.  **Update Login Logic (`web/src/pages/login.tsx`):**
    *   Refactor the form submission handler.
    *   **Await** the `signIn` response.
    *   **Check:** If the response contains an `error` object, **stop**. Do not redirect. Display the error message to the user (e.g., using a state variable to show red text or an alert).
    *   **Success:** Only redirect to the documentation URL if the response is successful.
    *   **UX:** Add a 'Loading...' state to the button while the request is in flight to prevent double-clicks.

2.  **Debug Navbar Session (`web/src/components/AuthNavbarItem.tsx`):**
    *   Add a console log inside the component to print the current `session`, `loading`, and `error` states from the `useSession` hook.
    *   Ensure the component explicitly renders the user's name when `session` is valid, and the 'Login' button when it is null.
    *   Ensure the Logout button triggers a page reload or state update after signing out to clear the UI."

## Response snapshot

Fixed login error handling and session feedback issues to prevent blind redirects and improve UX:

### 1. **Updated `web/src/pages/login.tsx`** (Login Error Handling)

**Problem:** Login was redirecting immediately after `await` without checking if authentication succeeded.

**Solution:** Capture response and check for `error` object before redirecting.

**Changes:**
- Line 39: Capture `signUp` response: `const response = await authClient.signUp.email(...)`
- Lines 49-53: Check for signup error:
  ```typescript
  if (response.error) {
    setError(response.error.message || 'Sign up failed');
    setLoading(false);
    return;
  }
  ```
- Line 61: Capture `signIn` response: `const response = await authClient.signIn.email(...)`
- Lines 67-71: Check for login error:
  ```typescript
  if (response.error) {
    setError(response.error.message || 'Login failed. Please check your credentials.');
    setLoading(false);
    return;
  }
  ```
- Lines 78-81: Improved catch block to reset loading state

**UX Improvements:**
- Loading state already implemented (line 203): `{loading ? 'Loading...' : ...}`
- Error display already implemented (lines 192-196): Red error message box
- Now errors are properly shown instead of blind redirect

### 2. **Updated `web/src/components/AuthNavbarItem.tsx`** (Session Debugging & Logout)

**Problem:** No visibility into session state, logout didn't clear UI properly.

**Solution:** Add debug logging and page reload on logout.

**Changes:**
- Line 14: Extract `error` from `useSession`: `const { data: session, isPending, error } = useSession()`
- Lines 17-24: Added debug logging with `useEffect`:
  ```typescript
  React.useEffect(() => {
    console.log('🔐 AuthNavbarItem Debug:', {
      session: session,
      isPending: isPending,
      error: error,
      user: session?.user,
    });
  }, [session, isPending, error]);
  ```
- Line 11: Added `ExecutionEnvironment` import
- Lines 42-48: Improved logout handler:
  ```typescript
  onClick={async () => {
    await signOut();
    // Reload page after logout to clear UI state
    if (ExecutionEnvironment.canUseDOM) {
      window.location.href = '/';
    }
  }}
  ```

**Benefits:**
- Console logs show session state changes in real-time
- Logout now redirects to home page, clearing all session-dependent UI
- Explicit rendering logic for logged-in vs logged-out states

### Flow Improvements:

**Before:**
```
User enters wrong password → signIn() → Redirects to /docs/en/intro → User sees error or nothing
```

**After:**
```
User enters wrong password → signIn() → Check response.error → Show error message → Stay on login page
```

**Logout Before:**
```
Click Logout → signOut() → UI state may be stale
```

**Logout After:**
```
Click Logout → signOut() → Redirect to home page → Fresh page load with no session
```

## Outcome

- ✅ Impact: **High** - Fixes critical UX issue where users can't see login errors
- 🧪 Tests: None (UI/UX improvement)
- 📁 Files: 2 modified (login.tsx, AuthNavbarItem.tsx)
- 🔁 Next prompts: Test with invalid credentials, verify session persistence across page reloads, add "Forgot Password" flow
- 🧠 Reflection: Better-Auth returns response objects with optional `error` field instead of throwing exceptions. Always check `response.error` before assuming success. Page reload on logout is a simple but effective way to ensure clean state.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (UX fix)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add toast notifications instead of inline error messages for better UX
