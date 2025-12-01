# Physical AI & Humanoid Robotics Textbook Platform

A hardware-adaptive online textbook platform for Physical AI & Humanoid Robotics that personalizes content based on users' hardware profiles and provides RAG-based chatbot assistance.

## 📋 Overview

This monorepo contains three services:
- **`/web`**: Docusaurus-based documentation frontend
- **`/api`**: FastAPI backend with RAG-powered Q&A
- **`/auth`**: Better-auth authentication service

### Key Features

- 🎯 **Hardware-Adaptive Content**: Personalizes instructions for RTX4090, Jetson, Laptop, or Cloud
- 🤖 **RAG-Powered Chatbot**: AI assistant answers questions using textbook content
- 🌍 **Bilingual Support**: Content in English and Urdu
- 🔐 **Secure Authentication**: User profiles with hardware background tracking
- 📚 **4 Core Modules**: ROS 2, Simulation, Isaac Sim, Vision-Language-Action

## 🚀 Quick Start

### Prerequisites

- **Python 3.10+** (for API service)
- **Node.js 18+** (for Auth & Web services)
- **PostgreSQL** (Neon recommended)
- **Qdrant** (Cloud or self-hosted)
- **Gemini API Key** (from Google AI Studio)

### 1. Environment Setup

```bash
# Clone the repository
git clone <your-repo-url>
cd code

# Copy environment template
cp .env.example .env

# Edit .env with your credentials
# - DATABASE_URL: Neon Postgres connection string
# - QDRANT_URL and QDRANT_API_KEY: From Qdrant Cloud
# - GEMINI_API_KEY: From Google AI Studio
# - JWT_SECRET: Generate with `openssl rand -base64 32`
```

### 2. Install Dependencies

```bash
# API Service (Python)
cd api
pip install -r requirements.txt

# Auth Service (Node.js)
cd ../auth
npm install

# Web Service (Docusaurus)
cd ../web
npm install
```

### 3. Database Setup

```bash
# Initialize Auth database schema
cd auth
npm run db:push

# Populate Qdrant with textbook content
cd ../api
python scripts/ingest.py
```

### 4. Start Services

Open three terminal windows:

```bash
# Terminal 1: API Service (port 8000)
cd api
uvicorn src.main:app --reload

# Terminal 2: Auth Service (port 3001)
cd auth
npm run dev

# Terminal 3: Web Service (port 3000)
cd web
npm start
```

Access the platform at **http://localhost:3000**

## 📁 Project Structure

```
code/
├── .env.example           # Environment configuration template
├── README.md              # This file
├── course_syllabus.md     # Source content for 4 modules
│
├── api/                   # FastAPI Backend
│   ├── src/
│   │   ├── main.py        # FastAPI app with /chat and /personalize endpoints
│   │   ├── agent.py       # RAG agent with Gemini 1.5 Flash
│   │   └── core/
│   │       └── config.py  # Pydantic settings
│   ├── scripts/
│   │   ├── ingest.py      # Qdrant ingestion script
│   │   ├── validate_links.py   # Link validation tool
│   │   └── lint_code.py   # Code block validation tool
│   └── requirements.txt   # Python dependencies
│
├── auth/                  # Better-Auth Service
│   ├── src/
│   │   ├── index.ts       # Auth server (port 3001)
│   │   ├── auth.config.ts # Better-auth configuration
│   │   └── db/
│   │       └── schema.ts  # Drizzle ORM schema (hardware_bg field)
│   ├── drizzle.config.ts  # Database migration config
│   └── package.json       # Node dependencies
│
└── web/                   # Docusaurus Frontend
    ├── docs/
    │   ├── en/            # English documentation
    │   │   ├── ros2/
    │   │   ├── simulation/
    │   │   ├── isaac-sim/
    │   │   └── vla/
    │   └── ur/            # Urdu documentation (parallel structure)
    ├── src/
    │   ├── components/
    │   │   ├── ChatWidget.tsx          # Floating chat assistant
    │   │   └── PersonalizeBtn.tsx      # Hardware personalization button
    │   ├── theme/
    │   │   └── DocItem/Layout/         # Custom Docusaurus layout
    │   └── utils/
    │       └── api.ts                   # API client utility
    ├── docusaurus.config.js  # Docusaurus configuration
    └── package.json          # Node dependencies
```

## 🔧 Configuration

### Environment Variables

See `.env.example` for all available configuration options.

**Required**:
- `DATABASE_URL`: PostgreSQL connection string
- `QDRANT_URL`: Qdrant endpoint
- `QDRANT_API_KEY`: Qdrant authentication
- `GEMINI_API_KEY`: Google Gemini API key

**Optional**:
- `API_PORT`: Backend port (default: 8000)
- `AUTH_PORT`: Auth service port (default: 3001)
- `DEBUG`: Enable debug mode (default: True)

### Service URLs

- **API**: http://localhost:8000
  - Health: `GET /api/health`
  - Chat: `POST /chat`
  - Personalize: `POST /personalize`

- **Auth**: http://localhost:3001
  - Health: `GET /api/auth/health`
  - Signup: `POST /api/auth/signup`
  - Login: `POST /api/auth/login`

- **Web**: http://localhost:3000

## 📚 API Documentation

### Chat Endpoint

```bash
POST /chat
Content-Type: application/json

{
  "message": "What is ROS 2?",
  "history": []  # Optional conversation history
}

# Response
{
  "response": "ROS 2 is...",
  "sources": [
    {
      "filename": "fundamentals.md",
      "module": "ros2",
      "title": "ROS 2 Fundamentals",
      "score": 0.95
    }
  ]
}
```

### Personalize Endpoint

```bash
POST /personalize
Content-Type: application/json

{
  "content": "Install ROS 2 on your system...",
  "hardware_bg": "RTX4090"
}

# Response
{
  "personalized_content": "Install ROS 2 with GPU acceleration for RTX4090..."
}
```

## 🧪 Quality Assurance

### Link Validation

```bash
cd api
python scripts/validate_links.py
```

Validates all markdown links to ensure no broken references.

### Code Linting

```bash
cd api
python scripts/lint_code.py
```

Validates Python code blocks in documentation for syntax errors.

## 🗄️ Database Schema

### User Table

```typescript
{
  id: string (UUID)
  email: string (unique)
  password: string (hashed)
  hardware_bg: "RTX4090" | "Jetson" | "Laptop" | "Cloud"
  software_bg: string
  createdAt: timestamp
  updatedAt: timestamp
}
```

## 🚢 Deployment

### Frontend (Vercel)

```bash
cd web
npm run build

# Deploy to Vercel
vercel deploy
```

### Backend (Render)

1. Create new Web Service on Render
2. Connect repository
3. Set build command: `pip install -r requirements.txt`
4. Set start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables from `.env`

### Auth (Render)

1. Create new Web Service on Render
2. Connect repository
3. Set build command: `npm install`
4. Set start command: `npm start`
5. Add environment variables from `.env`

## 🔐 Security

- ✅ All secrets in `.env` (never committed)
- ✅ HTTP-only session tokens
- ✅ Password hashing via better-auth
- ✅ CORS configured for localhost
- ✅ Pydantic validation on all API inputs
- ⚠️ Change `JWT_SECRET` in production
- ⚠️ Restrict CORS origins in production

## 📊 Tech Stack

### Backend
- **FastAPI**: High-performance Python API framework
- **Google Gemini 1.5 Flash**: LLM for RAG and personalization
- **Qdrant**: Vector database for textbook embeddings
- **OpenAI Python Client**: Gemini compatibility layer
- **Pydantic**: Data validation

### Authentication
- **Better-Auth**: Modern authentication library
- **Drizzle ORM**: Type-safe database operations
- **Neon Postgres**: Serverless PostgreSQL
- **Node.js HTTP**: Native server (no Express)

### Frontend
- **Docusaurus 3**: Documentation framework
- **React 18**: UI library
- **TypeScript**: Type safety
- **CSS Modules**: Component styling

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run quality checks:
   ```bash
   python api/scripts/validate_links.py
   python api/scripts/lint_code.py
   ```
5. Submit a pull request

## 📄 License

[Your License Here]

## 🆘 Support

- **Documentation Issues**: Check link validation output
- **API Errors**: Check FastAPI logs at `http://localhost:8000/docs`
- **Auth Issues**: Verify `DATABASE_URL` in `.env`
- **Build Errors**: Ensure all dependencies are installed

## 🗺️ Roadmap

- [ ] Complete User Story 1: Signup UI
- [ ] Complete User Story 2: Personalization backend
- [ ] Complete User Story 4: Language switching component
- [ ] Add automated tests
- [ ] Performance optimization
- [ ] Production deployment

---

**Built with ❤️ for the Physical AI & Humanoid Robotics community**
