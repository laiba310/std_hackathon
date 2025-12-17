import sys
import os

print("🔍 Checking Environment...")

# 1. Check Email Validator
try:
    import email_validator
    print("✅ Library: email-validator is INSTALLED.")
except ImportError:
    print("❌ ERROR: email-validator is MISSING!")
    print("   Run: pip install email-validator")

# 2. Check Pydantic Schema
try:
    sys.path.append(os.getcwd())
    from backend.schemas import UserCreate
    user = UserCreate(email="test@example.com", password="123", experience_level="Beginner")
    print("✅ Code: Schemas are working correctly!")
except Exception as e:
    print(f"❌ ERROR in Code: {e}")

print("--------------------------------")