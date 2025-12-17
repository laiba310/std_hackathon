# backend/schemas.py
from pydantic import BaseModel, EmailStr
from typing import Optional

# Base Schema
class UserBase(BaseModel):
    email: EmailStr

# Signup ke liye (User jo data bhejega)
class UserCreate(UserBase):
    password: str
    experience_level: Optional[str] = None  # optional; values like "Beginner" / "Programmer"

# Login Token ke liye
class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    username: Optional[str] = None

# Browser ko wapis dikhane ke liye (Response)
class UserResponse(UserBase):
    id: int
    is_active: bool = True
    experience_level: Optional[str] = None

    class Config:
        from_attributes = True   # important so SQLAlchemy models map to Pydantic models

class TranslationRequest(BaseModel):
    text: str
    target_lang: str = "Urdu"

class ChatRequest(BaseModel):
    question: str
