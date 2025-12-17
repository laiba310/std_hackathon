import sys
import os

# Add the root folder to system path
sys.path.append(os.getcwd())

from backend.database import engine, Base
from backend.models import User

print("🔌 Connecting to Database...")

try:
    # Try creating tables
    Base.metadata.create_all(bind=engine)
    print("✅ SUCCESS! Tables created successfully.")
    print("Database is connected and ready.")
except Exception as e:
    print("\n❌ ERROR: Could not connect to Database.")
    print(f"Reason: {e}")