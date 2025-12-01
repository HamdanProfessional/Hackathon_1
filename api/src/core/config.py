"""
Configuration Management for AI Robotics Platform API
Loads environment variables using Pydantic BaseSettings
"""
from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables"""

    # Database Configuration
    DATABASE_URL: str

    # Qdrant Configuration
    QDRANT_URL: str
    QDRANT_API_KEY: str

    # Google Gemini API (via OpenAI compatibility)
    OPENAI_API_BASE: str
    GEMINI_API_KEY: str

    # Application Settings
    API_HOST: str = "0.0.0.0"
    API_PORT: int = 8000
    DEBUG: bool = False

    # CORS Settings
    CORS_ORIGINS: list[str] = ["http://localhost:3000", "http://localhost:3001"]

    class Config:
        env_file = ".env"
        case_sensitive = True


# Global settings instance
settings = Settings()
