---

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

---

/sp.specify "Project: Physical AI & Humanoid Robotics Textbook Platform.

1. **Content Source:** Use the provided `course_syllabus.md` as the absolute source of truth.
   - **Modules:** 1. ROS 2 (Nervous System), 2. Simulation (Digital Twin), 3. Isaac Sim (Brain), 4. VLA (Capstone).

2. **User Experience:**
   - **Signup:** Collect `hardware_bg` (RTX4090, Jetson, Laptop, Cloud) and `software_bg`.
   - **Reading:** Users see a 'Personalize' button on chapters. Clicking it adapts the guide to their hardware.
   - **Chat:** A floating widget that answers questions using the textbook content (RAG).

3. **Backend Services:**
   - **Auth:** Node.js server handling session management via `better-auth`.
   - **API:** Python FastAPI server handling:
     - `POST /chat`: RAG agent interaction.
     - `POST /personalize`: LLM content rewriting.
   - **Scripts:** `ingest.py` for Qdrant indexing and `validate_links.py` for maintenance."

---

/sp.clarify "1. **Out of Scope:**
   - Controlling physical robots (code only).
   - Payment processing.
   - Video hosting (use embeds).
   - Native mobile apps.

2. **Scale Assumptions:**
   - Hackathon Demo: <100 concurrent users.
   - Data: ~50 Markdown chapters.
   - Latency: Optimize for single-user speed (Gemini Flash).

3. **Translation Logic:**
   - Do NOT translate dynamically via API (too slow/costly).
   - Pre-generate Urdu markdown files during the build phase.

4. **Security:**
   - Never commit `.env`.
   - Use Pydantic models for all API inputs."

---

/sp.plan "Phase 1: Foundation
1. **Scaffold:** Create directories `/web`, `/api`, `/auth`.
2. **Config:** Setup `.env` and `requirements.txt`.
3. **DB:** Connect Neon and Qdrant.

Phase 2: The Book (Content)
4. **English:** Generate Modules 1-4 in `/web/docs/en` based on Syllabus.
5. **Urdu:** Translate and save to `/web/docs/ur`.
6. **Config:** Setup Docusaurus i18n or folder routing.

Phase 3: The Brain (Backend)
7. **Ingestion:** Script to chunk/embed docs to Qdrant.
8. **Agent:** Implement `api/agent.py` using `openai-agents`.
9. **Endpoints:** `/chat`, `/personalize`.

Phase 4: The Experience (Frontend/Auth)
10. **Auth:** Setup Better-Auth schema (`hardware_bg`).
11. **UI:** Create `PersonalizeBtn`, `LangSwitch`, `ChatWidget`.
12. **Integration:** Embed in Docusaurus Layout.

Phase 5: Bonus Skills
13. **Scripts:** Create `link_checker.py` and `code_linter.py`."

---

/sp.checklist "1. **Repo:** Monorepo structure created?
2. **Content:** Are all 4 Modules present in English and Urdu?
3. **Database:** Is `hardware_bg` column present in User table?
4. **RAG:** Does `ingest.py` populate Qdrant successfully?
5. **Chat:** Does the Agent answer using textbook context?
6. **Personalization:** Does the button rewrite text based on hardware profile?
7. **Deployment:** Is the build valid for Vercel/Render?"

---

/sp.tasks "1. Create `.env` file with provided API keys.
2. Initialize Docusaurus in `/web` and install dependencies.
3. Initialize FastAPI in `/api` with `main.py`, `agent.py`, `core/config.py`.
4. Initialize Node project in `/auth` with `better-auth` and `pg`.
5. Read `course_syllabus.md` and generate 5 Markdown chapters in `/web/docs/en/`.
6. Translate those 5 chapters to `/web/docs/ur/`.
7. Write `scripts/ingest.py` to index content to Qdrant.
8. Implement `api/agent.py` with `search_textbook` tool and `openai-agents`.
9. Implement React components: `PersonalizeButton.tsx`, `ChatWidget.tsx`.
10. Swizzle Docusaurus `DocItem` to include the new buttons."

---

/sp.analyze "Verify coverage of the 300-point Hackathon requirements:
1. **Textbook Creation:** Covered by Phase 2 (Content Gen).
2. **RAG Chatbot:** Covered by Phase 3 (Agent + Qdrant).
3. **Bonus 1 (Reusable Skills):** Covered by Phase 5 (`scripts/`).
4. **Bonus 2 (Signup/Auth):** Covered by Phase 4 (Better-Auth + Hardware Background).
5. **Bonus 3 (Personalization):** Covered by Phase 3 & 4 (API + Button).
6. **Bonus 4 (Urdu):** Covered by Phase 2 & 4 (Static Files + Switcher).

---

/sp.implement "Execute Phase 1 & 2.
1. **Environment:** Create the `.env` file with the keys provided in history.
2. **Scaffold:** Create the `/web`, `/api`, and `/auth` folders.
3. **Content:** Read `course_syllabus.md` and generate the English and Urdu markdown files in `web/docs/`.
   - **Crucial:** Ensure code blocks in Urdu files remain in English.
4. **Backend Base:** Create `api/main.py` and `api/agent.py`. Setup the `AsyncOpenAI` client using `GEMINI_API_KEY` and `OPENAI_API_BASE` from env."

---

/sp.implement "Phase 3: The Brain (Backend RAG & Agent)
1.  **Dependencies:** Ensure `qdrant-client`, `openai-agents`, and `fastapi` are installed in the Python environment.
2.  **Ingestion Script:** Create `scripts/ingest.py`.
    -   **Logic:** Recursively read all `.md` files in `web/docs/en/`.
    -   **Chunking:** Split text by headers or 500-character windows.
    -   **Embedding:** Use the configured OpenAI/Gemini client to generate embeddings.
    -   **Storage:** Upsert vectors + metadata (filename, header) to Qdrant collection `robotics_textbook`.
3.  **Agent Tool:** In `api/agent.py`, define a function `search_textbook(query: str)` that performs a cosine similarity search on Qdrant and returns the top 3 chunks.
4.  **Agent Setup:** Initialize the `openai-agents` `Agent` class:
    -   **Model:** `google/gemini-1.5-flash` (via OpenAI compatibility layer).
    -   **Instructions:** 'You are a robotics tutor. Use search_textbook to answer questions. Be concise.'
    -   **Tools:** Attach `[search_textbook]`.
5.  **API Endpoint:** Update `api/main.py` to include `POST /chat`.
    -   **Input:** `{ message: str, history: list }`.
    -   **Action:** Invoke `agent.run()`.
    -   **Output:** `{ response: str }`."

---

/sp.implement "Phase 4a: Authentication Infrastructure
1.  **Node Service:** In `/auth`, ensure `package.json` includes `better-auth`, `pg`, and `dotenv`.
2.  **Schema Configuration:** Create `/auth/auth.config.ts`.
    -   **Database:** Connect to the Neon Postgres URL from `.env`.
    -   **User Model:** Extend the default schema to include `hardware_bg` (String/Enum: 'RTX4090', 'Jetson', 'Laptop', 'Cloud').
    -   **Providers:** Enable `username/password` or `email/password` credential provider.
3.  **Server:** Create `/auth/server.ts` to run the auth server on port 3001 (or separate route if using Next.js, but standard Node server for this stack).
4.  **Database Sync:** Run the better-auth migration command to push the schema to Neon."

---

/sp.implement "Phase 4b: Frontend Interactivity & Integration
1.  **API Client:** Create `web/src/utils/api.ts` to handle requests to the Python API (`localhost:8000`) and Auth API (`localhost:3001`). Include interceptors to attach the session token if available.
2.  **Personalize Component:** Create `web/src/components/PersonalizeButton.tsx`.
    -   **UI:** A button labeled 'Personalize for [User Hardware]'.
    -   **Logic:** On click, extract the main markdown content from the DOM. `POST` to `api/personalize` with `{ content, hardware_bg }`.
    -   **Effect:** Replace the DOM text with the streaming response from the LLM.
3.  **Chat Widget:** Create `web/src/components/ChatWidget.tsx`.
    -   **UI:** Floating Action Button (bottom-right). Expands to a chat window.
    -   **Logic:** Maintain local chat state. `POST` to `api/chat`. Display 'Thinking...' while waiting.
4.  **Docusaurus Integration:**
    -   **Swizzle:** Run `docusaurus swizzle @docusaurus/theme-classic DocItem/Layout --wrap`.
    -   **Wrapper:** Edit `web/src/theme/DocItem/Layout/index.js` (or `.tsx`).
    -   **Render:** Inject `<PersonalizeButton />` at the top of the content area and `<ChatWidget />` globally."

---

/sp.implement "Phase 5: Bonus Skills (Quality Assurance)
1.  **Link Validator:** Create `scripts/validate_links.py`.
    -   **Goal:** Ensure no broken relative links in the documentation.
    -   **Logic:** Regex scan all `[text](path)` in `web/docs/`. Check if the target file exists. Print a report.
2.  **Code Linter:** Create `scripts/lint_code.py`.
    -   **Goal:** Ensure code blocks in the textbook are valid.
    -   **Logic:** Extract python code blocks from Markdown. Run `pylint --errors-only` on the extracted strings. Report syntax errors.
3.  **Final Verification:** Run `scripts/ingest.py` to ensure the vector DB is populated, then run `npm build` in `/web` to verify the static site generation succeeds."

---

/sp.implement "Execute the full End-to-End Implementation Plan (Phases 1-5) to establish the Monorepo, Backend, Content pipeline, and Frontend integration.

**Action: Generates the following file structure and codebase:**

### 1. Infrastructure & Config (Phase 1)
*   **Create Root Files:**
    *   `.env.example`: Template for keys (`GEMINI_API_KEY`, `OPENAI_API_BASE`, `DATABASE_URL`, `QDRANT_URL`, `BETTER_AUTH_SECRET`).
    *   `requirements.txt`: Python deps (`fastapi`, `uvicorn`, `openai-agents`, `qdrant-client`, `asyncpg`, `python-dotenv`).
    *   `auth/package.json`: Node deps (`better-auth`, `express`, `pg`, `cors`, `dotenv`).
*   **Scaffold Commands:** Assume directories `/api`, `/auth`, `/web` (Docusaurus), and `/scripts` exist.

### 2. Content Generation Pipeline (Phase 2)
*   **Create `scripts/scaffold_content.py`:**
    *   Define a list of 4 Modules (ROS2, Simulation, Isaac Sim, VLA).
    *   Function to generate 4 Markdown files in `web/docs/en/` with proper Docusaurus frontmatter and placeholders.
    *   Function to generate 4 Markdown files in `web/docs/ur/` that translates descriptions to pseudo-Urdu but **strictly preserves** English technical terms and code blocks.

### 3. The Brain: Backend & Agent (Phase 3)
*   **Create `api/core/config.py`:** Load env vars using Pydantic Settings.
*   **Create `api/agent.py`:**
    *   Initialize `openai-agents` SDK with `Google Gemini 1.5 Flash` (via OpenAI compatibility).
    *   Define `Agent` with instructions: 'You are a Robotics Professor. Answer based on the textbook.'
    *   (Placeholder): Define `search_textbook` tool skeleton.
*   **Create `api/main.py`:**
    *   `POST /chat`: Route user queries to `agent.run()`.
    *   `POST /personalize`: Route content + `hardware_bg` to LLM for rewriting.
*   **Create `scripts/ingest.py`:**
    *   Script to read all `.md` files in `web/docs/en/`.
    *   Chunk text and upsert to Qdrant Vector DB.

### 4. Authentication & Frontend (Phase 4)
*   **Create `auth/auth.config.ts`:**
    *   Config `better-auth` with Postgres.
    *   Add custom User schema field: `hardware_bg` (Enum: Laptop, Jetson, Cloud).
*   **Create `auth/server.ts`:** Express server mounting the Auth handler on port 3001.
*   **Create `web/src/utils/api.ts`:** Axios/Fetch wrapper for API (8000) and Auth (3001).
*   **Create `web/src/components/PersonalizeBtn.tsx`:** React component that calls `/api/personalize` and updates the DOM.
*   **Create `web/src/components/ChatWidget.tsx`:** Floating UI that calls `/api/chat`.

### 5. Quality Assurance (Phase 5)
*   **Create `scripts/validate_links.py`:** Scans Markdown for broken relative links.

**Constraint:** Output the full code for these files. For standard scaffolding (like Docusaurus init), provide the shell commands."

---

/sp.checklist "Final Verification:
1. Does the Monorepo structure match the Spec?
2. Are English and Urdu docs present?
3. Does `scripts/ingest.py` run without errors?
4. Does the `/chat` endpoint respond using the Agent?
5. Does the Auth server start and connect to Postgres?
6. Does the Docusaurus build succeed with the custom components?"

---

/sp.implement "Phase 5: Auth Integration & Context-Aware Chat

**Goal:** Enable user login to save 'Hardware Preferences' and implement a 'Highlight-to-Chat' workflow.

### 1. Auth Client Setup (`web/src/lib/auth-client.ts`)
*   **Action:** Initialize the Better-Auth React client.
*   **Code:**
    ```typescript
    import { createAuthClient } from 'better-auth/react';
    export const authClient = createAuthClient({
        baseURL: 'http://localhost:3001' // Point to Auth Server
    });
    ```

### 2. Login & Onboarding Page (`web/src/pages/login.tsx`)
*   **UI:** Create a clean Login/Signup form.
*   **Fields:** Email, Password, Name.
*   **Preferences (Crucial):** Add a Dropdown for `hardware_bg` (Options: 'RTX 4090', 'Jetson Orin', 'Laptop CPU', 'Google Colab').
*   **Logic:** On submit, call `authClient.signUp.email`. Redirect to `/docs/en/01-ros2-nervous-system` upon success.

### 3. Global Chat State (`web/src/context/ChatContext.tsx`)
*   **Action:** Create a React Context to manage the Chat State globally.
*   **State:**
    *   `isChatOpen`: boolean.
    *   `activeContexts`: Array of strings (snippets of text selected by the user).
    *   `addContext(text)`: Adds text to the array.
    *   `removeContext(index)`: Removes text.
*   **Provider:** Wrap the application in `web/src/theme/Root.tsx`.

### 4. 'Select-to-Ask' Logic (`web/src/theme/DocItem/Layout/index.tsx`)
*   **Action:** Swizzle or Wrap the DocItem Layout to detect text selection.
*   **Logic:**
    *   Add a `onMouseUp` event listener to the content area.
    *   If `window.getSelection().toString()` is > 0:
        *   Show a small 'Tooltip/Button' near the cursor labeled '💬 Add to Chat'.
    *   **On Click:** Call `addContext(selection)` and open the Chat Widget.

### 5. Update Chat Widget (`web/src/components/ChatWidget.tsx`)
*   **UI Update:** Above the input box, render 'Context Chips' for every item in `activeContexts`.
    *   Style: Small pill tags with an 'X' button to remove them.
*   **API Logic:** When sending a message to `POST /chat`:
    *   Include `context: activeContexts.join('\n')` in the payload.
    *   Clear context after sending (optional, or keep until manually cleared).

### 6. Personalization Hook
*   **Update `PersonalizeBtn.tsx`:** Instead of asking for hardware manually, try to read `authClient.useSession()`.
    *   If logged in: Use `session.user.hardware_bg`.
    *   If not logged in: Fallback to a prompt or default."

---

/sp.implement "Fix Translation Routing & Sidebar Logic

**Goal:** Strictly separate English (`/docs/en/`) and Urdu (`/docs/ur/`) navigation and add a toggle button.

### 1. Split Sidebars (`web/sidebars.ts` or `.js`)
*   **Problem:** Currently `dirName: '.'` shows all files mixed together.
*   **Action:** Define two distinct sidebars.
*   **Code:**
    ```javascript
    const sidebars = {
      // Sidebar for English docs
      englishSidebar: [
        {
          type: 'autogenerated',
          dirName: 'en', // Only scan web/docs/en
        },
      ],
      // Sidebar for Urdu docs
      urduSidebar: [
        {
          type: 'autogenerated',
          dirName: 'ur', // Only scan web/docs/ur
        },
      ],
    };
    export default sidebars;
    ```

### 2. Create Language Switcher (`web/src/components/LangSwitcher.tsx`)
*   **Action:** Create a component that toggles the current URL between `/en/` and `/ur/`.
*   **Logic:**
    *   Get current path (e.g., `/docs/en/01-ros2`).
    *   If path contains `/en/`, render button 'اردو' -> Link to `/docs/ur/01-ros2`.
    *   If path contains `/ur/`, render button 'English' -> Link to `/docs/en/01-ros2`.
    *   If path is not in docs, hide or default to English.

### 3. Add to Navbar (`web/docusaurus.config.ts`)
*   **Action:** Register the `LangSwitcher` as a custom navbar item.
*   **Config Update:**
    *   In `themeConfig.navbar.items`:
        *   Add a new item: `{ type: 'custom-langSwitcher', position: 'right' }`.
*   **Component Registration:** You must register this custom type in `web/src/theme/NavbarItem/ComponentTypes.tsx` (if using Swizzling) OR simply add it as a standard HTML link if that's too complex, BUT better:
    *   **Simpler Alternative:** Just add the component to the `Root.tsx` or inject it via `NavbarItem` swizzling.
    *   **Instruction for Claude:** 'Swizzle the NavbarItem to support the custom component OR just insert the logic into `web/src/theme/Navbar/Content/index.tsx` if strictly necessary. However, the cleanest way for this stack is to create a **NavbarItem** component.'

    *   **Refined Instruction:**
        1. Create `web/src/components/LangSwitcher.tsx`.
        2. Create `web/src/theme/NavbarItem/index.tsx` (Swizzle wrapper).
        3. Inside the wrapper, check if the item type is `'custom-langSwitcher'`. If so, render your component. Otherwise, render the default.

### 4. Default Route Redirect
*   **Action:** Ensure clicking 'Docs' in the navbar goes to `/docs/en/01-ros2-nervous-system` (English default) instead of the generic `/docs/intro`."

---

/sp.implement "Refine Personalization: Add Beginner/Advanced Tiers

**Goal:** Improve content adaptation by adding a 'Skill Level' dimension. 'Beginner' should simplify concepts; 'Advanced' should go deep into technical implementation.

### 1. Update Auth Schema (`auth/auth.config.ts`)
*   **Action:** Add a new field to `user.additionalFields`.
*   **Field:** `skill_level`
    *   Type: `string`
    *   Input: `true`
    *   Default: `'Beginner'`
    *   Options: `['Beginner', 'Advanced']`
*   **Migration:** Run `npx @better-auth/cli generate` inside `/auth` after changing the config.

### 2. Update Signup UI (`web/src/pages/login.tsx`)
*   **Action:** Add a dropdown for 'Skill Level' in the signup form.
*   **Logic:** Pass `skill_level` to the `authClient.signUp.email` function.

### 3. Update Personalization API (`api/main.py`)
*   **Action:** Update `POST /personalize` to accept `skill_level` in the payload.
*   **Prompt Engineering:** Update the LLM system prompt to handle the tiers:
    *   **If Beginner:** 'Explain like I am 12. Use analogies. Avoid complex math. Focus on the "Why".'
    *   **If Advanced:** 'Assume expert knowledge. Use industry jargon. Focus on performance, optimization, and "How". Show code implementation details.'

### 4. Update Frontend Button (`web/src/components/PersonalizeBtn.tsx`)
*   **Action:** Retrieve `session.user.skill_level`.
*   **Payload:** Send `{ content, hardware_bg, skill_level }` to the API.
*   **UI:** Update the button text to say 'Personalize (Beginner)' or 'Personalize (Advanced)' based on their profile."

---

/sp.implement "Frontend: Integrate Better-Auth Login Button in Navbar

**Goal:** Add a dynamic authentication button to the Docusaurus Navbar using the Better-Auth React client.

### 1. Create Auth Component (`web/src/components/AuthNavbarItem.tsx`)
*   **Action:** Create a React component that uses the Better-Auth session hook.
*   **Imports:** `import { authClient } from '../lib/auth-client';`
*   **Logic:**
    ```tsx
    import React from 'react';
    import { authClient } from '../lib/auth-client';
    import Link from '@docusaurus/Link';

    export default function AuthNavbarItem() {
      const { data: session, isPending } = authClient.useSession();

      if (isPending) return <div className='navbar__item'>Loading...</div>;

      if (session) {
        return (
          <div className='navbar__item'>
            <span style={{marginRight: '10px'}}>Hi, {session.user.name}</span>
            <button
              className='button button--secondary button--sm'
              onClick={() => authClient.signOut()}
            >
              Logout
            </button>
          </div>
        );
      }

      return (
        <Link to='/login' className='button button--primary button--sm navbar__item'>
          Login
        </Link>
      );
    }
    ```

### 2. Register Custom Navbar Type (`web/src/theme/NavbarItem/ComponentTypes.tsx`)
*   **Context:** Docusaurus requires mapping custom types to components.
*   **Action:** Create this file to register our component.
*   **Code:**
    ```tsx
    import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';
    import AuthNavbarItem from '@site/src/components/AuthNavbarItem';

    export default {
      ...ComponentTypes,
      'custom-better-auth': AuthNavbarItem, // Registering the type
    };
    ```

### 3. Update Config (`web/docusaurus.config.ts`)
*   **Action:** Add the new item to the Navbar array.
*   **Location:** `themeConfig.navbar.items`.
*   **Entry:**
    ```javascript
    {
      type: 'custom-better-auth', // Must match the key in ComponentTypes
      position: 'right',
    }
    ```

### 4. Verify Auth Client (`web/src/lib/auth-client.ts`)
*   **Check:** Ensure this file exists and exports `authClient` initialized with `baseURL: 'http://localhost:3001'` (or your auth server URL)."

---

/sp.implement "Fix Docusaurus Navbar Registration Error

**Goal:** Align the Custom Navbar Item registration so Docusaurus can find the `AuthNavbarItem` component.

### 1. Fix Registration (`web/src/theme/NavbarItem/ComponentTypes.tsx`)
*   **Action:** Create or Overwrite this file. It **must** map the specific string used in your config to the React component.
*   **Code:**
    ```tsx
    import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';
    import AuthNavbarItem from '@site/src/components/AuthNavbarItem';

    export default {
      ...ComponentTypes,
      'custom-authNavbarItem': AuthNavbarItem, // <--- This Key MUST match 'type' in config
    };
    ```

### 2. Verify Component Export (`web/src/components/AuthNavbarItem.tsx`)
*   **Action:** Ensure the component is exported as `default`.
*   **Check:**
    ```tsx
    export default function AuthNavbarItem() { ... }
    ```

### 3. Verify Config (`web/docusaurus.config.ts`)
*   **Action:** Ensure the `type` string matches exactly.
*   **Check:**
    ```javascript
    items: [
      // ...
      {
        type: 'custom-authNavbarItem', // Matches the key in ComponentTypes.tsx
        position: 'right',
      },
    ]
    ```

---

/sp.implement "Fix Auth Server Startup & Build Scripts

1. **Update `package.json` Scripts:**
   - **Build:** Ensure there is a build script: `"build": "tsc"`.
   - **Start:** Update the start script to match your source file name. If your source is `src/server.ts`, the compiled file will be `dist/server.js`.
     - Change `"start": "node dist/index.js"` to `"start": "node dist/server.js"`.
   - **Dev:** Ensure `"dev": "tsx watch src/server.ts"` exists.

2. **Compile:**
   - Run `npm run build` to generate the `dist/` folder."

```

/sp.implement "Fix Production Env Loading & Rebuild

Goal: Ensure .env is loaded before the app crashes, and rebuild the stale dist folder.

1. Update Entry Point (src/index.ts or src/server.ts)

Action: Open your main entry file.

Critical Change: Move import 'dotenv/config'; to the very first line of the file. It must be before any other imports like ./auth.config or express.

Debug: Add this line immediately after the import:

console.log("🔍 Checking DB URL:", process.env.DATABASE_URL ? "Loaded ✅" : "Missing ❌");
2. Force Rebuild

Context: You are running node dist/index.js, but your changes are in src/.

Action: Run the build command to update the dist folder.

Command: npm run build

3. Execution

Action: Run npm start again.

Expected Output: You should see '🔍 Checking DB URL: Loaded ✅' in the console."

---

/sp.implement "Implement Content Personalization Logic.

1. **Backend (`api/main.py`):**
   - Update `POST /personalize` to accept `{ content, hardware, skill }`.
   - Use `client.chat.completions.create` (Gemini) to rewrite the text.
   - System Prompt: 'Rewrite the following robotics technical documentation for a [skill] level student using [hardware]. Adjust complexity and implementation details accordingly.'

2. **Frontend (`web/src/components/PersonalizeBtn.tsx`):**
   - Use `authClient.useSession()` to get user preferences.
   - On click:
     1. Select the main content: `const text = document.querySelector('article')?.innerText;`
     2. Show 'Rewriting...' state.
     3. POST to `/personalize` with session data.
     4. Replace the article content with the result (using a Markdown renderer or simple text replacement)."

---

/sp.implement "Fix CORS Wildcard Error in Auth Server

**Goal:** Resolve the browser blocking requests due to invalid CORS configuration for credentials.

**Context:** The error `The value of the 'Access-Control-Allow-Origin' header... must not be the wildcard '*'` indicates that the Auth server is allowing *all* origins, but the client is trying to send cookies (`credentials: include`). Browsers strictly forbid this combination.

**Action:** Update the CORS middleware in `auth/src/server.ts` (or `index.ts`):

1.  **Remove Wildcard:** You must stop using `*` or the default settings for `origin`.
2.  **Set Explicit Origin:** Change the `origin` option to be an explicit array containing the frontend URL: `['http://localhost:3000']`.
3.  **Enable Credentials:** Ensure `credentials: true` is explicitly set in the CORS options.
4.  **Verification:** Ensure this middleware is applied **before** any route handlers."

---

/sp.implement "Debug Backend LLM Connection.

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

---

/sp.implement "Fix LLM Connection Protocol Error

**Goal:** Fix the `OPENAI_API_BASE` URL format to include the required `https://` protocol.

**Diagnosis:** The error `Request URL is missing an 'http://' or 'https://' protocol` means the app is trying to connect to `generativelanguage.googleapis.com...` instead of `https://generativelanguage.googleapis.com...`.

**Action:**

1.  **Update `.env`:**
    -   Rewrite the `.env` file to ensure `OPENAI_API_BASE` is exactly:
        `OPENAI_API_BASE=https://generativelanguage.googleapis.com/v1beta/openai/`
    -   *Crucial:* Ensure you do not delete the existing `GEMINI_API_KEY` or `DATABASE_URL`.

2.  **Harden Config (`api/src/core/config.py`):**
    -   Update the `Settings` class to automatically fix this in the future.
    -   Add a Pydantic validator (or `__init__` logic):
        ```python
        @field_validator("OPENAI_API_BASE")
        def check_protocol(cls, v):
            if v and not v.startswith("http"):
                return f"https://{v}"
            return v
        ```
    -   *Note:* If using Pydantic v1, use `@validator`. For v2, use `@field_validator`."

---