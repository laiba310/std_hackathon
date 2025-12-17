import sys
import os
# Path fix
sys.path.append(os.getcwd())

from backend.database import engine, Base
from backend.models import User

print("--- Dropping old tables (Deleting users)...")
try:
    Base.metadata.drop_all(bind=engine)
    print("--- Old tables deleted.")
except Exception as e:
    print(f"--- Error dropping tables: {e}")

print("--- Creating new tables (With experience_level)...")
try:
    Base.metadata.create_all(bind=engine)
    print("--- SUCCESS! New fresh database created.")
except Exception as e:
    print(f"--- Error creating tables: {e}")