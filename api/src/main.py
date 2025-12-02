"""
FastAPI Main Application
Physical AI & Humanoid Robotics Textbook Platform - API Service
"""
# CRITICAL: Load .env file FIRST before any other imports
from dotenv import load_dotenv
from pathlib import Path

# Load .env from repository root (two levels up from this file)
env_path = Path(__file__).parent.parent.parent / ".env"
print(f"🔍 Loading .env from: {env_path}")
print(f"🔍 .env exists: {env_path.exists()}")
load_dotenv(dotenv_path=env_path, override=True)

import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from core.config import settings
from router import route_request

# Debug: Verify configuration on startup
print("=" * 60)
print("🔧 API Configuration Debug")
print("=" * 60)
print(f"DEBUG: OPENAI_API_BASE = {os.getenv('OPENAI_API_BASE')}")
print(f"DEBUG: GEMINI_API_KEY Loaded = {bool(os.getenv('GEMINI_API_KEY'))}")
print(f"DEBUG: OPENAI_API_KEY Loaded = {bool(os.getenv('OPENAI_API_KEY'))}")
print(f"DEBUG: DATABASE_URL Loaded = {bool(os.getenv('DATABASE_URL'))}")
print(f"DEBUG: QDRANT_URL = {os.getenv('QDRANT_URL')}")
print(f"DEBUG: QDRANT_API_KEY Loaded = {bool(os.getenv('QDRANT_API_KEY'))}")
print("=" * 60)

# Initialize FastAPI application
app = FastAPI(
    title="Physical AI Textbook API",
    description="Backend API for hardware-adaptive robotics textbook platform",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models for chat endpoint
class ChatRequest(BaseModel):
    """Request model for chat endpoint"""
    message: str
    history: Optional[List[Dict[str, str]]] = None

class ChatResponse(BaseModel):
    """Response model for chat endpoint"""
    response: str
    sources: Optional[List[Dict[str, Any]]] = None
    agent_type: Optional[str] = None
    intent: Optional[str] = None

# Pydantic models for personalization endpoint
class PersonalizeRequest(BaseModel):
    """Request model for personalization endpoint"""
    content: str
    hardware_bg: str
    skill_level: str = "Beginner"

class PersonalizeResponse(BaseModel):
    """Response model for personalization endpoint"""
    personalized_content: str


# Health check endpoint
@app.get("/api/health")
async def health_check():
    """Health check endpoint for monitoring"""
    return {
        "status": "healthy",
        "service": "api",
        "version": "1.0.0"
    }

# Root endpoint
@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "Physical AI Textbook API",
        "docs": "/api/docs"
    }

# Chat endpoint - Multi-Agent RAG-based Q&A
@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Multi-Agent RAG-based chat endpoint using specialized agents
    """
    try:
        result = await route_request(
            query=request.message,
            history=request.history
        )

        return ChatResponse(
            response=result["response"],
            sources=result.get("sources", []),
            agent_type=result.get("agent_type"),
            intent=result.get("intent")
        )
    except Exception as e:
        return ChatResponse(
            response=f"Error processing question: {str(e)}",
            sources=[],
            agent_type="Error",
            intent="ERROR"
        )

# Personalization endpoint - Hardware and skill level adaptation
@app.post("/personalize", response_model=PersonalizeResponse)
async def personalize(request: PersonalizeRequest):
    """
    Personalize textbook content based on hardware and skill level
    """
    try:
        from openai import AsyncOpenAI

        # Initialize OpenAI client (using Gemini via compatibility layer)
        client = AsyncOpenAI(
            api_key=settings.GEMINI_API_KEY,
            base_url=settings.OPENAI_API_BASE
        )

        # Build prompt based on skill level
        if request.skill_level == "Beginner":
            skill_instruction = """
Adapt this content for a BEGINNER learner:
- Explain like I am 12 years old
- Use analogies and real-world examples
- Avoid complex mathematical formulas
- Focus on the "Why" and conceptual understanding
- Break down technical jargon into simple terms
- Keep code examples basic and well-commented
"""
        else:  # Advanced
            skill_instruction = """
Adapt this content for an ADVANCED learner:
- Assume expert knowledge of robotics and programming
- Use industry-standard terminology and jargon
- Focus on performance optimization and "How" it works internally
- Show detailed code implementation with advanced patterns
- Include technical details about algorithms and data structures
- Discuss edge cases and production considerations
"""

        # Build hardware-specific instruction
        hardware_instruction = f"""
Adapt code examples and performance guidance for: {request.hardware_bg}
- Adjust code snippets for {request.hardware_bg} constraints
- Mention hardware-specific optimizations
- Update performance expectations for this hardware
"""

        system_prompt = f"""You are a technical content adapter for a robotics textbook.

{skill_instruction}

{hardware_instruction}

Important:
- Preserve all markdown formatting
- Keep the same overall structure
- Only modify explanations and code examples
- Maintain technical accuracy
"""

        # Call LLM to personalize content
        # UPDATED: Using Gemini 2.5 Flash explicitly
        response = await client.chat.completions.create(
            model="gemini-2.5-flash", 
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Adapt this content:\n\n{request.content}"}
            ],
            temperature=0.7,
            max_tokens=2000
        )

        personalized_text = response.choices[0].message.content

        return PersonalizeResponse(
            personalized_content=personalized_text
        )

    except Exception as e:
        # Debugging print to help you see if it fails
        print(f"❌ Personalization Error: {e}")
        return PersonalizeResponse(
            personalized_content=f"Error personalizing content: {str(e)}"
        )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host=settings.API_HOST,
        port=settings.API_PORT,
        reload=settings.DEBUG
    )