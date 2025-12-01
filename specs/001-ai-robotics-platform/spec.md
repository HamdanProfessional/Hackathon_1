# Feature Specification: Physical AI & Humanoid Robotics Textbook Platform

**Feature Branch**: `001-ai-robotics-platform`
**Created**: 2025-12-01
**Status**: Draft
**Input**: User description: "Project: Physical AI & Humanoid Robotics Textbook Platform. 1. **Content Source:** Use the provided `course_syllabus.md` as the absolute source of truth. - **Modules:** 1. ROS 2 (Nervous System), 2. Simulation (Digital Twin), 3. Isaac Sim (Brain), 4. VLA (Capstone). 2. **User Experience:** - **Signup:** Collect `hardware_bg` (RTX4090, Jetson, Laptop, Cloud) and `software_bg`. - **Reading:** Users see a 'Personalize' button on chapters. Clicking it adapts the guide to their hardware. - **Chat:** A floating widget that answers questions using the textbook content (RAG). 3. **Backend Services:** - **Auth:** Node.js server handling session management via `better-auth`. - **API:** Python FastAPI server handling: - `POST /chat`: RAG agent interaction. - `POST /personalize`: LLM content rewriting. - **Scripts:** `ingest.py` for Qdrant indexing and `validate_links.py` for maintenance."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New Student Signup (Priority: P1)

A prospective student visiting the platform for the first time creates an account to access the Physical AI textbook content. During signup, they provide their hardware background (RTX4090, Jetson, Laptop, or Cloud) and software experience level to enable personalized learning experiences.

**Why this priority**: This is the entry point for all users and foundational to personalization. Without capturing hardware context during signup, the platform's core value proposition (hardware-specific guidance) cannot function.

**Independent Test**: A new user can navigate to the signup page, complete the registration form including hardware and software background selections, and successfully create an account. The system stores this profile information for subsequent personalization.

**Acceptance Scenarios**:

1. **Given** a visitor lands on the signup page, **When** they provide name, email, password, select "RTX4090" as hardware background and "Intermediate" as software background, **Then** an account is created and they can immediately log in to access content.
2. **Given** a user attempts signup, **When** they skip the hardware background selection, **Then** the system displays a validation error requiring hardware selection before proceeding.
3. **Given** a user completes signup with "Jetson" hardware, **When** they log in and view their profile, **Then** the profile correctly displays "Jetson" as their hardware background.

---

### User Story 2 - Personalized Chapter Reading (Priority: P2)

A logged-in student reading a chapter on ROS 2 installation clicks the "Personalize" button. The chapter content dynamically updates to show installation instructions, code examples, and performance considerations specific to their hardware profile (e.g., Jetson shows ARM-specific commands, Laptop users see CPU-only alternatives).

**Why this priority**: This is the platform's core differentiator - hardware-adaptive learning content. It directly addresses the challenge that robotics tutorials often assume specific hardware configurations.

**Independent Test**: A logged-in user with a defined hardware background can view any chapter, click "Personalize," and observe content changes that reference their specific hardware. The personalization should modify both instructional text and code snippets.

**Acceptance Scenarios**:

1. **Given** a user with "Laptop" hardware profile is viewing the "Isaac Sim Setup" chapter, **When** they click the "Personalize" button, **Then** the content updates to show cloud-based alternatives and warns about GPU limitations.
2. **Given** a user with "RTX4090" hardware is viewing a simulation chapter, **When** they personalize the content, **Then** code examples include high-performance rendering settings appropriate for their GPU.
3. **Given** a user has personalized a chapter, **When** they click the "Personalize" button again, **Then** the content reverts to the original generic version.
4. **Given** a user personalizes content, **When** they navigate to a different chapter and return, **Then** the personalization state resets (not persisted across navigation).

---

### User Story 3 - Interactive Chatbot Assistance (Priority: P2)

A student struggling with a ROS 2 concept opens the floating chat widget, types "How do I create a ROS 2 publisher in Python?", and receives a concise answer with code examples sourced from the textbook chapters. They can optionally highlight text on the page to provide context for their question.

**Why this priority**: On-demand assistance reduces learning friction and helps students get unstuck without leaving the platform. RAG-based answers ensure accuracy by grounding responses in the textbook content.

**Independent Test**: A logged-in user can open the chat widget from any chapter page, ask a question related to the course syllabus content, and receive a relevant answer within 3 seconds. The chatbot should cite which chapter or module the answer comes from.

**Acceptance Scenarios**:

1. **Given** a user is on the "ROS 2 Nodes" chapter, **When** they open chat and ask "What is a ROS 2 node?", **Then** the chatbot returns a definition sourced from that chapter with a reference citation.
2. **Given** a user highlights the text "URDF format" on a page, **When** they open chat and ask "Explain this", **Then** the chatbot uses the highlighted context to provide a targeted explanation of URDF.
3. **Given** a user asks "How do I build a time machine?", **When** the chatbot processes the question, **Then** it responds with "I can only answer questions based on the Physical AI & Humanoid Robotics textbook content."
4. **Given** a user asks about "Gazebo physics simulation", **When** the chatbot responds, **Then** the answer includes a link to the relevant chapter for deeper reading.

---

### User Story 4 - Bilingual Content Access (Priority: P3)

A student whose primary language is Urdu navigates to the textbook and clicks the language toggle. The interface switches to Urdu, and the URL changes from `/docs/en/module-1` to `/docs/ur/module-1`, displaying the same content translated into Urdu while preserving technical terms.

**Why this priority**: Language accessibility expands the platform's reach to non-English-speaking robotics students. This is lower priority than core functionality but critical for inclusive education.

**Independent Test**: A user can toggle between English and Urdu at any point in their reading journey. The language preference persists across page navigation within the same session.

**Acceptance Scenarios**:

1. **Given** a user is viewing `/docs/en/ros2-basics`, **When** they click the language toggle to Urdu, **Then** the page navigates to `/docs/ur/ros2-basics` with content in Urdu.
2. **Given** content is displayed in Urdu, **When** technical terms like "ROS 2", "URDF", or "Isaac Sim" appear, **Then** they remain in English as standard technical terminology.
3. **Given** a user has selected Urdu, **When** they navigate to a new chapter, **Then** the new chapter loads in Urdu automatically.
4. **Given** a chapter exists in English but the Urdu translation is missing, **When** a user toggles to Urdu, **Then** a message displays: "This chapter is currently available in English only" with a link to the English version.

---

### Edge Cases

- What happens when a user changes their hardware background after signup? Should the platform allow profile updates, and if so, how does this affect previously viewed content?
- How does the system handle network latency for the `/personalize` endpoint? If personalization takes >5 seconds, should there be a loading state or timeout?
- What if the `course_syllabus.md` file is corrupted or missing required module sections? Should there be validation during content ingestion?
- How does the chatbot handle ambiguous questions that could apply to multiple modules (e.g., "What is simulation?")? Should it ask for clarification or return multiple answers?
- What happens when a user tries to personalize content while not logged in? Should they see an error, a login prompt, or a preview of personalization benefits?
- If Qdrant vector database is unavailable, should the chat widget gracefully degrade or display an error message?
- How should the system handle users on mobile devices where `window.getSelection()` behaves differently?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST use `course_syllabus.md` as the exclusive source of truth for all textbook content generation.
- **FR-002**: Content MUST be organized into four primary modules: ROS 2 (Nervous System), Simulation (Digital Twin), Isaac Sim (Brain), and VLA (Capstone).
- **FR-003**: The signup process MUST collect two pieces of user information: `hardware_bg` (RTX4090, Jetson, Laptop, or Cloud) and `software_bg` (experience level).
- **FR-004**: A "Personalize" button MUST be visible and accessible on all chapter reading pages for authenticated users.
- **FR-005**: Clicking the "Personalize" button MUST trigger dynamic content adaptation based on the user's stored `hardware_bg`, modifying both instructional text and code examples.
- **FR-006**: Personalized content MUST revert to the original generic version when the "Personalize" button is clicked a second time (toggle behavior).
- **FR-007**: A floating chat widget MUST be accessible on all chapter pages, allowing users to ask questions without navigating away.
- **FR-008**: The chat widget MUST answer questions using Retrieval-Augmented Generation (RAG) exclusively from textbook content indexed in Qdrant.
- **FR-009**: The chat widget MUST support text selection context injection via `window.getSelection()` to allow users to ask about highlighted passages.
- **FR-010**: The system MUST provide bilingual content in English (`/docs/en/`) and Urdu (`/docs/ur/`) with URL-based language switching.
- **FR-011**: Language switching MUST operate via URL path changes only (static routing). Urdu translations MUST be pre-generated during the build phase, not translated dynamically at runtime.
- **FR-012**: Technical terminology (ROS 2, URDF, Isaac Sim, etc.) MUST remain in English even in Urdu-language content.
- **FR-013**: An `ingest.py` script MUST be provided to populate the Qdrant vector database from `course_syllabus.md`.
- **FR-014**: A `validate_links.py` script MUST be provided to check for broken internal and external links in the content.
- **FR-015**: The system MUST cite source chapters or modules when the chatbot provides answers.
- **FR-016**: All API input data MUST be validated using structured validation models (e.g., Pydantic schemas) before processing.
- **FR-017**: Environment configuration files (`.env`) MUST never be committed to version control and MUST be listed in `.gitignore`.

### Key Entities

- **User**: Represents a student using the platform. Key attributes include unique identifier, email, authentication credentials, `hardware_bg` (RTX4090, Jetson, Laptop, Cloud), and `software_bg` (experience level).

- **Module**: Represents a top-level curriculum section from `course_syllabus.md`. The platform has exactly four modules: ROS 2, Simulation, Isaac Sim, and VLA. Each module contains multiple chapters (approximately 50 chapters total across all modules).

- **Chapter**: Represents a specific learning unit within a module (e.g., "Setting up ROS 2 Workspace"). Chapters contain the actual instructional content, code examples, and exercises. Each chapter exists in both English and Urdu versions.

- **PersonalizationContext**: Represents the user-specific parameters used for content adaptation. Includes `hardware_bg` from the user profile and the original chapter content. This is a transient entity used during personalization requests.

- **ChatMessage**: Represents a single question-answer exchange in the chatbot. Contains the user's question text, optional `selection_context` (highlighted text), the generated answer, source citations (chapter references), and timestamp.

- **ContentEmbedding**: Represents a vector embedding of textbook content stored in Qdrant. Each embedding corresponds to a chunk of a chapter, with metadata including module name, chapter title, language (en/ur), and source location.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 90% of new users successfully complete the signup process including hardware background selection without assistance or errors.

- **SC-002**: Users can personalize chapter content and see visible changes within 3 seconds of clicking the "Personalize" button.

- **SC-003**: The chatbot provides relevant answers (as rated by users) to at least 85% of questions that fall within the textbook's scope.

- **SC-004**: At least 75% of users who personalize content report in surveys that the adapted instructions were clearer for their hardware setup than generic tutorials.

- **SC-005**: The system supports at least 100 concurrent users reading and personalizing content without performance degradation (hackathon demo scale).

- **SC-006**: Language switching between English and Urdu occurs instantly (< 500ms) as it is URL-based navigation.

- **SC-007**: The chat widget responds to user questions within 3 seconds for 95% of queries under normal load.

- **SC-008**: Users complete at least one full module (reading all chapters) at a 40% higher rate compared to traditional static documentation platforms.

## Clarifications

### Session 2025-12-01

- Q: What is explicitly out of scope for this platform? → A: Controlling physical robots (code/simulation only), payment processing, video hosting (use embeds), and native mobile apps.
- Q: What are the scale assumptions for the initial deployment? → A: Hackathon demo targeting <100 concurrent users, ~50 Markdown chapters, optimized for single-user speed using Gemini Flash.
- Q: How should Urdu translation be implemented? → A: Pre-generate Urdu markdown files during the build phase (NOT dynamic API translation due to cost/latency).
- Q: What are the critical security requirements? → A: Never commit `.env` files, use Pydantic models for all API input validation.

## Out of Scope

The following features are explicitly **NOT** included in this platform:

- **Physical robot control**: The platform provides code examples and simulation guidance only. Users cannot control actual hardware robots through the web interface.
- **Payment processing**: All content is freely accessible. No subscription, paywall, or e-commerce features.
- **Video hosting**: Educational videos must be embedded from external platforms (YouTube, Vimeo). No native video upload or streaming.
- **Native mobile apps**: The platform is web-only (responsive design). No iOS/Android native applications.

## Assumptions

- Users have basic familiarity with robotics concepts or are willing to learn from foundational chapters.
- The platform assumes users are honest about their hardware background during signup (no verification mechanism).
- Internet connectivity is required for all features; there is no offline mode.
- The English content is the primary source; Urdu translations are pre-generated during the build phase and reviewed for accuracy before deployment.
- Users understand that personalization is session-based and does not persist across page reloads or navigation.
- The chatbot's knowledge is limited to the textbook content; it cannot answer general robotics questions outside the syllabus.
- Code examples assume users have basic command-line proficiency appropriate for their software background level.
- **Scale**: Initial deployment targets hackathon demo scale (<100 concurrent users, ~50 chapters total across all modules).
- **Performance**: System optimized for single-user speed rather than high-throughput multi-tenancy.
