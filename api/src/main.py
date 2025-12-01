"""
FastAPI Main Application
Physical AI & Humanoid Robotics Textbook Platform - API Service
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from core.config import settings
from agent import textbook_agent

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

# Chat endpoint - RAG-based Q&A
@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    RAG-based chat endpoint using textbook content

    Processes student questions using Retrieval-Augmented Generation:
    1. Query is sent to the agent
    2. Agent uses search_textbook tool to find relevant content from Qdrant
    3. Agent generates response based on retrieved content
    4. Returns answer with source citations

    Args:
        request: ChatRequest with message and optional conversation history

    Returns:
        ChatResponse with agent response and source citations from textbook
    """
    try:
        result = await textbook_agent.query(
            message=request.message,
            history=request.history
        )

        return ChatResponse(
            response=result["response"],
            sources=result.get("sources", [])
        )
    except Exception as e:
        return ChatResponse(
            response=f"Error processing question: {str(e)}",
            sources=[]
        )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host=settings.API_HOST,
        port=settings.API_PORT,
        reload=settings.DEBUG
    )
