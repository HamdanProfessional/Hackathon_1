"""
Vercel Serverless Entry Point for FastAPI
Lightweight wrapper that only imports what's needed
"""
import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from mangum import Mangum

# Create FastAPI app with minimal config
app = FastAPI(
    title="Physical AI Textbook API",
    version="1.0.0"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for now
    allow_origin_regex=r"https://.*\.vercel\.app",
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Health check endpoint (no dependencies)
@app.get("/api/health")
async def health_check():
    return {
        "status": "healthy",
        "service": "api",
        "version": "1.0.0"
    }

# Root endpoint
@app.get("/")
async def root():
    return {
        "message": "Physical AI Textbook API",
        "docs": "/api/docs"
    }

# Chat endpoint - lazy load heavy dependencies
@app.post("/chat")
async def chat(request: dict):
    # Only import when actually needed
    try:
        from src.router import route_request

        result = await route_request(
            query=request.get("message", ""),
            history=request.get("history", [])
        )

        return {
            "response": result["response"],
            "sources": result.get("sources", []),
            "agent_type": result.get("agent_type"),
            "intent": result.get("intent")
        }
    except Exception as e:
        return {
            "response": f"Error: {str(e)}",
            "sources": [],
            "agent_type": "error",
            "intent": "ERROR"
        }

# Personalization endpoint - lazy load
@app.post("/personalize")
async def personalize(request: dict):
    try:
        from openai import AsyncOpenAI

        client = AsyncOpenAI(
            api_key=os.getenv("GEMINI_API_KEY"),
            base_url=os.getenv("OPENAI_API_BASE")
        )

        content = request.get("content", "")
        skill_level = request.get("skill_level", "Beginner")
        hardware_bg = request.get("hardware_bg", "")

        system_prompt = f"""Adapt this content for a {skill_level} learner with {hardware_bg} hardware background."""

        response = await client.chat.completions.create(
            model="gemini-2.0-flash-exp",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Adapt:\n\n{content}"}
            ],
            temperature=0.7,
            max_tokens=2000
        )

        return {
            "personalized_content": response.choices[0].message.content
        }
    except Exception as e:
        return {
            "personalized_content": f"Error: {str(e)}"
        }

# Export app for Vercel (Vercel uses ASGI directly with FastAPI)
# No need for Mangum wrapper
