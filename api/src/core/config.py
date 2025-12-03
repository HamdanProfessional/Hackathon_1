"""
Configuration Management for AI Robotics Platform API
Loads environment variables using Pydantic BaseSettings
"""
from typing import List, Optional
from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict

class Settings(BaseSettings):
    """Application settings loaded from environment variables"""

    # Database Configuration
    DATABASE_URL: str = Field(..., description="PostgreSQL connection string")

    # Qdrant Configuration (Optional for local development)
    QDRANT_URL: Optional[str] = None
    QDRANT_API_KEY: Optional[str] = None

    # OpenAI API Configuration
    OPENAI_API_KEY: str = Field(default="", description="OpenAI API key for embeddings")

    # Google Gemini API (via OpenAI compatibility)
    OPENAI_API_BASE: str = Field(default="", description="OpenAI API base URL for Gemini")
    GEMINI_API_KEY: str = Field(default="", description="Google Gemini API key for LLM")

    # Application Settings
    API_HOST: str = Field(default="0.0.0.0", description="API host address")
    API_PORT: int = Field(default=8000, description="API port")
    DEBUG: bool = Field(default=False, description="Debug mode")

    # CORS Settings
    CORS_ORIGINS: List[str] = Field(
        default=["http://localhost:3000", "http://localhost:3001"],
        description="Allowed CORS origins"
    )

    @field_validator("OPENAI_API_BASE")
    @classmethod
    def check_protocol(cls, v: str) -> str:
        """Ensure OPENAI_API_BASE has http:// or https:// protocol"""
        if not v:
            print("[WARNING] OPENAI_API_BASE is empty!")
            return v

        if not v.startswith(("http://", "https://")):
            print("[FIX] Auto-fixing OPENAI_API_BASE: adding https:// prefix")
            print(f"   Before: {v}")
            fixed = f"https://{v}"
            print(f"   After:  {fixed}")
            return fixed

        print(f"[OK] OPENAI_API_BASE validated: {v}")
        return v

    model_config = SettingsConfigDict(
        env_file=(".env", "../.env"),
        env_file_encoding="utf-8",
        case_sensitive=True,
        extra="ignore"
    )

# Global settings instance
settings = Settings()