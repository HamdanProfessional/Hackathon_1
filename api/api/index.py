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
    allow_origins=[
        "http://localhost:3000",
        "https://hamdanprofessional.github.io",
    ],
    allow_origin_regex=r"https://.*\.(vercel\.app|github\.io)",
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

# Chat endpoint - simplified for serverless (no RAG for now)
@app.post("/chat")
async def chat(request: dict):
    try:
        from openai import AsyncOpenAI

        client = AsyncOpenAI(
            api_key=os.getenv("GEMINI_API_KEY"),
            base_url=os.getenv("OPENAI_API_BASE")
        )

        message = request.get("message", "")
        history = request.get("history", [])

        # Build conversation messages
        messages = [
            {"role": "system", "content": "You are a helpful tutor for Physical AI and Robotics. Answer questions about ROS2, Isaac Sim, Gazebo, and humanoid robotics."}
        ]

        # Add history
        for msg in history[-5:]:  # Last 5 messages for context
            messages.append(msg)

        # Add current message
        messages.append({"role": "user", "content": message})

        # Call LLM
        response = await client.chat.completions.create(
            model="gemini-2.5-flash",
            messages=messages,
            temperature=0.7,
            max_tokens=1000
        )

        return {
            "response": response.choices[0].message.content,
            "sources": [],
            "agent_type": "Tutor",
            "intent": "GENERAL"
        }
    except Exception as e:
        import traceback
        error_details = traceback.format_exc()
        print(f"ERROR: {error_details}")
        return {
            "response": f"I apologize, but I'm having trouble connecting right now. Error: {str(e)}",
            "error_details": error_details,
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
            model="gemini-2.5-flash",
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
