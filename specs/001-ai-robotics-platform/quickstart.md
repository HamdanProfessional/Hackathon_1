# Quickstart Guide

**Date**: 2025-11-30
**Status**: Draft

This guide provides instructions for setting up and running the project locally for development.

## Prerequisites
- Node.js (LTS version)
- Python 3.10+
- Git
- Access to Neon and Qdrant Cloud accounts.

## 1. Initial Setup

### Clone the Repository
This project is self-contained. Once you have access, navigate to the project directory.

### Environment Variables
1. Copy the `.env.example` file (if it exists) to a new file named `.env` at the repository root.
2. Fill in the required secrets and configuration values:
   ```env
   # Neon Postgres Connection String
   DATABASE_URL="..."

   # Qdrant Cloud URL and API Key
   QDRANT_URL="..."
   QDRANT_API_KEY="..."

   # Gemini API Key (via OpenAI compatibility)
   OPENAI_API_BASE="https://generativelanguage.googleapis.com/v1beta"
   GEMINI_API_KEY="..."
   ```

## 2. Install Dependencies

### Backend Services
```bash
# Install dependencies for the API service
cd api
pip install -r requirements.txt

# Install dependencies for the Auth service
cd ../auth
npm install
```

### Frontend
```bash
# Install dependencies for the Docusaurus web app
cd ../web
npm install
```

## 3. Prepare Content

### Run Ingestion Script
To populate the Qdrant vector database with the textbook content, run the ingestion script from the repository root:
```bash
python scripts/ingest.py
```
*Note: This requires `course_syllabus.md` to be present at the root.*

## 4. Run the Application

The application consists of three separate services that must be run concurrently.

### Terminal 1: Auth Service
```bash
cd auth
npm start
```

### Terminal 2: API Service
```bash
cd api
uvicorn src.main:app --reload
```

### Terminal 3: Frontend Service
```bash
cd web
npm start
```

The application should now be running locally. The Docusaurus frontend will typically be available at `http://localhost:3000`.
