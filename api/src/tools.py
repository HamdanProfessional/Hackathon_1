"""
Shared Tools for Multi-Agent System
Provides common utilities that can be used by multiple specialized agents
"""
from openai import AsyncOpenAI
from qdrant_client import QdrantClient
from typing import List, Dict, Any
from src.core.config import settings

# Initialize clients
embedding_client = AsyncOpenAI(
    api_key=settings.OPENAI_API_KEY
)

# Initialize Qdrant client only if URL is configured
qdrant_client = None
if settings.QDRANT_URL and settings.QDRANT_API_KEY:
    qdrant_client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY
    )


def search_textbook(query: str, top_k: int = 3) -> List[Dict[str, Any]]:
    """
    Search textbook content using RAG (Retrieval-Augmented Generation)

    This tool is shared across all specialized agents (Tutor, Quiz, Coder)
    to provide consistent access to textbook knowledge.

    Args:
        query: Search query from user
        top_k: Number of results to return (default: 3)

    Returns:
        List of relevant textbook chunks with metadata
        Returns empty list if Qdrant is not configured
    """
    # Handle case when Qdrant is not configured (local dev)
    if not qdrant_client:
        print("Warning: Qdrant not configured, returning empty results")
        return []

    try:
        # Generate embedding for the query using OpenAI
        embedding_response = embedding_client.embeddings.create(
            model="text-embedding-3-small",
            input=query
        )
        query_vector = embedding_response.data[0].embedding

        # Search Qdrant
        search_results = qdrant_client.search(
            collection_name="robotics_textbook",
            query_vector=query_vector,
            limit=top_k
        )

        # Format results
        results = []
        for result in search_results:
            results.append({
                "text": result.payload.get("text", ""),
                "filename": result.payload.get("filename", "unknown"),
                "module": result.payload.get("module", "unknown"),
                "title": result.payload.get("title", ""),
                "score": result.score
            })

        return results

    except Exception as e:
        print(f"Error in search_textbook: {e}")
        return []


# Tool definition for function calling (shared across agents)
SEARCH_TEXTBOOK_TOOL = {
    "type": "function",
    "function": {
        "name": "search_textbook",
        "description": "Search the Physical AI & Robotics textbook for relevant content. Use this to find information before answering student questions.",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "The search query based on the student's question"
                },
                "top_k": {
                    "type": "integer",
                    "description": "Number of results to return (default 3)",
                    "default": 3
                }
            },
            "required": ["query"]
        }
    }
}
