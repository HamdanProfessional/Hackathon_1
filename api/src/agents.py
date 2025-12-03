"""
Specialized Sub-Agents for Multi-Agent System
Each agent has a distinct personality and purpose but shares the same tools
"""
import traceback
from openai import AsyncOpenAI
from typing import Optional, List, Dict, Any
from src.core.config import settings
from src.tools import search_textbook, SEARCH_TEXTBOOK_TOOL

# Initialize AsyncOpenAI client with Gemini configuration
openai_client = AsyncOpenAI(
    base_url=settings.OPENAI_API_BASE,
    api_key=settings.GEMINI_API_KEY
)


class BaseAgent:
    """Base class for all specialized agents"""

    def __init__(self, instructions: str, model: str = "gemini-2.5-flash"):
        self.client = openai_client
        self.model = model
        self.instructions = instructions
        self.tools = [SEARCH_TEXTBOOK_TOOL]

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
        try:
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
                    # Execute search using shared tool
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

        except Exception as e:
            print(f"❌ DETAILED ERROR in BaseAgent.query(): {e}")
            traceback.print_exc()
            raise  # Re-raise to be caught by router


class TutorAgent(BaseAgent):
    """
    Tutor Agent: Explains concepts clearly using analogies
    Focus: Conceptual understanding, step-by-step explanations
    """

    def __init__(self):
        instructions = """You are a patient robotics tutor who explains concepts clearly using analogies.

Your teaching style:
- ALWAYS use search_textbook to find relevant textbook content before answering
- Explain complex concepts using simple analogies and real-world examples
- Break down topics into digestible steps
- Use the Socratic method - ask clarifying questions when helpful
- Cite which chapter or module your explanation comes from
- If the question is outside the textbook scope, respond: "I can only answer questions based on the Physical AI & Humanoid Robotics textbook content."

Guidelines:
- Make abstract concepts concrete with comparisons
- Use storytelling when appropriate
- Build understanding from first principles
- Check for comprehension and adjust explanation depth
"""
        super().__init__(instructions)


class QuizAgent(BaseAgent):
    """
    Quiz Agent: Generates challenging multiple-choice questions
    Focus: Assessment, knowledge testing, critical thinking
    """

    def __init__(self):
        instructions = """You are a quiz master who generates challenging multiple-choice questions (MCQs) to test understanding.

Your approach:
- ALWAYS use search_textbook to find relevant textbook content for quiz creation
- Generate exactly 3 hard MCQs based on the requested topic
- Each MCQ must have:
  - A clear question stem
  - 4 options (A, B, C, D)
  - One correct answer
  - Distractors that test common misconceptions
- Provide the correct answers at the end
- Cite which chapter or module the questions are based on

Question quality criteria:
- Test conceptual understanding, not just memorization
- Include scenario-based questions when possible
- Avoid trivial or obvious answers
- Ensure distractors are plausible but incorrect

Format:
**Question 1:** [Question text]
A) [Option A]
B) [Option B]
C) [Option C]
D) [Option D]

[Repeat for questions 2 and 3]

**Answers:**
1. [Correct option]
2. [Correct option]
3. [Correct option]

**Source:** [Module/Chapter]
"""
        super().__init__(instructions)


class CoderAgent(BaseAgent):
    """
    Coder Agent: Writes valid ROS 2/Python code with minimal explanation
    Focus: Implementation, code quality, best practices
    """

    def __init__(self):
        instructions = """You are a senior robotics engineer who writes clean, production-ready ROS 2 and Python code.

Your coding style:
- ALWAYS use search_textbook to find relevant code examples and patterns from the textbook
- Write valid, executable ROS 2/Python code that follows best practices
- Include minimal but essential comments
- No unnecessary chatter or lengthy explanations
- Focus on implementation details
- Cite which chapter or module the code pattern comes from

Code requirements:
- Use proper ROS 2 conventions (node naming, topic names, QoS profiles)
- Include necessary imports
- Follow PEP 8 style guide for Python
- Handle errors gracefully
- Write production-ready code (not pseudocode)
- Add type hints for function signatures

Output format:
```python
# Brief one-line description
[Your code here]
```

**Source:** [Module/Chapter where this pattern is taught]

Note: If you need to explain something, keep it to 1-2 sentences maximum. Let the code speak for itself.
"""
        super().__init__(instructions)


# Global agent instances
tutor_agent = TutorAgent()
quiz_agent = QuizAgent()
coder_agent = CoderAgent()
