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
