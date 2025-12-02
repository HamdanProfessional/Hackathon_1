"""
AI Agent for Textbook Q&A
Uses openai-agents SDK with Google Gemini 1.5 Flash via OpenAI compatibility layer
Implements RAG (Retrieval-Augmented Generation) with Qdrant vector store
"""
import os
from openai import AsyncOpenAI
from qdrant_client import QdrantClient
from typing import Optional, List, Dict, Any
import asyncio

# Load settings
from core.config import settings

# Initialize AsyncOpenAI client with Gemini configuration (for LLM)
openai_client = AsyncOpenAI(
    base_url=settings.OPENAI_API_BASE,
    api_key=settings.GEMINI_API_KEY
)

# Initialize separate OpenAI client for embeddings
embedding_client = AsyncOpenAI(
    api_key=settings.OPENAI_API_KEY
)

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=settings.QDRANT_URL,
    api_key=settings.QDRANT_API_KEY
)


def search_textbook(query: str, top_k: int = 3) -> List[Dict[str, Any]]:
    """
    Search textbook content using RAG (Retrieval-Augmented Generation)

    Args:
        query: Search query from user
        top_k: Number of results to return (default: 3)

    Returns:
        List of relevant textbook chunks with metadata
    """
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


class TextbookAgent:
    """
    Agentic AI assistant for Physical AI & Robotics textbook
    Uses RAG pattern with Qdrant vector store and openai-agents
    """

    def __init__(self):
        self.client = openai_client
        self.model = "google/gemini-1.5-flash"  # Gemini model via OpenAI compatibility
        self.instructions = """You are a robotics tutor. Use search_textbook to answer questions. Be concise.

Guidelines:
- ALWAYS use search_textbook to find relevant textbook content before answering
- Cite which chapter or module your answer comes from
- If the question is outside the textbook scope, respond: "I can only answer questions based on the Physical AI & Humanoid Robotics textbook content."
- Provide code examples when relevant from the retrieved content
- Be concise but thorough
- Use technical terminology appropriately
"""

        # Tool definition for function calling
        self.tools = [
            {
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
        ]

    async def query(
        self,
        message: str,
        history: Optional[List[Dict[str, str]]] = None
    ) -> Dict[str, Any]:
        """
        Process a user question using the agentic AI pattern with RAG

        Args:
            message: User's question
            history: Conversation history (list of {"role": "user/assistant", "content": "..."})

        Returns:
            Dictionary with response and source citations
        """
        # Build messages array
        messages = [{"role": "system", "content": self.instructions}]

        # Add history if provided
        if history:
            messages.extend(history)

        # Add current message
        messages.append({"role": "user", "content": message})

        # First API call with tools
        response = await self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            tools=self.tools,
            tool_choice="auto",
            temperature=0.7,
            max_tokens=1000
        )

        response_message = response.choices[0].message
        tool_calls = response_message.tool_calls

        # If no tool calls, return direct response
        if not tool_calls:
            return {
                "response": response_message.content,
                "sources": []
            }

        # Execute tool calls
        messages.append(response_message)
        sources = []

        for tool_call in tool_calls:
            function_name = tool_call.function.name
            function_args = eval(tool_call.function.arguments)  # Parse JSON arguments

            if function_name == "search_textbook":
                # Execute search
                search_results = search_textbook(**function_args)
                sources.extend(search_results)

                # Format results for LLM
                function_response = "\n\n".join([
                    f"[{r['module']}] {r['title']}\n{r['text'][:500]}..."
                    for r in search_results
                ])

                # Add tool response to messages
                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "name": function_name,
                    "content": function_response
                })

        # Second API call with tool results
        final_response = await self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=0.7,
            max_tokens=1000
        )

        return {
            "response": final_response.choices[0].message.content,
            "sources": [
                {
                    "filename": s["filename"],
                    "module": s["module"],
                    "title": s["title"],
                    "score": s["score"]
                }
                for s in sources
            ]
        }


# Global agent instance
textbook_agent = TextbookAgent()
