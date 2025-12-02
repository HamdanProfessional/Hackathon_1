"""
Intent Router for Multi-Agent System
Uses lightweight LLM to classify user intent and route to appropriate specialized agent
"""
import traceback
from openai import AsyncOpenAI
from typing import Dict, Any, Optional, List
from core.config import settings
from agents import tutor_agent, quiz_agent, coder_agent

# Initialize lightweight OpenAI client for intent classification
intent_client = AsyncOpenAI(
    base_url=settings.OPENAI_API_BASE,
    api_key=settings.GEMINI_API_KEY
)


async def classify_intent(query: str) -> str:
    """
    Classify user intent using a lightweight LLM call

    Args:
        query: User's question or request

    Returns:
        Intent classification: 'TUTOR' | 'QUIZ' | 'CODE'
    """
    classification_prompt = """You are an intent classifier for a robotics education platform.

Classify the user's query into ONE of these categories:

1. **TUTOR** - User wants explanation, understanding, or conceptual help
   Examples:
   - "What is a ROS 2 node?"
   - "Can you explain SLAM to me?"
   - "How does sensor fusion work?"
   - "Why do we need DDS in ROS 2?"

2. **QUIZ** - User wants to test their knowledge or practice
   Examples:
   - "Give me some questions about ROS 2"
   - "Test my understanding of navigation"
   - "Quiz me on Isaac Sim"
   - "Can you create practice problems?"

3. **CODE** - User wants code implementation or examples
   Examples:
   - "Write a ROS 2 publisher node"
   - "Show me code for a subscriber"
   - "How do I implement a service in ROS 2?"
   - "Give me a code example for lidar processing"

Respond with ONLY one word: TUTOR, QUIZ, or CODE

User query: {query}

Classification:"""

    try:
        response = await intent_client.chat.completions.create(
            model="gemini-2.0-flash-exp",  # Fast model for classification
            messages=[
                {"role": "user", "content": classification_prompt.format(query=query)}
            ],
            temperature=0.3,  # Low temperature for consistent classification
            max_tokens=10
        )

        intent = response.choices[0].message.content.strip().upper()

        # Validate intent
        if intent not in ["TUTOR", "QUIZ", "CODE"]:
            # Default to TUTOR if classification is unclear
            print(f"Warning: Unclear intent classification '{intent}', defaulting to TUTOR")
            return "TUTOR"

        return intent

    except Exception as e:
        print(f"❌ DETAILED ERROR in intent classification: {e}")
        traceback.print_exc()
        # Default to TUTOR on error
        return "TUTOR"


async def route_request(
    query: str,
    history: Optional[List[Dict[str, str]]] = None
) -> Dict[str, Any]:
    """
    Route user request to appropriate specialized agent based on intent

    Args:
        query: User's question or request
        history: Optional conversation history

    Returns:
        Dictionary with response, sources, and agent type used
    """
    # Step 1: Classify intent
    intent = await classify_intent(query)
    print(f"🎯 Intent classified as: {intent}")

    # Step 2: Route to appropriate agent
    if intent == "TUTOR":
        agent = tutor_agent
        agent_type = "Tutor"
    elif intent == "QUIZ":
        agent = quiz_agent
        agent_type = "Quiz Master"
    else:  # CODE
        agent = coder_agent
        agent_type = "Code Expert"

    # Step 3: Execute agent query
    try:
        result = await agent.query(message=query, history=history)

        # Add agent type to response
        return {
            "response": result["response"],
            "sources": result.get("sources", []),
            "agent_type": agent_type,
            "intent": intent
        }

    except Exception as e:
        print(f"❌ DETAILED ERROR in agent query: {e}")
        traceback.print_exc()
        return {
            "response": f"Error processing request: {str(e)}",
            "sources": [],
            "agent_type": agent_type,
            "intent": intent
        }
