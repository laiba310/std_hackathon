# backend/main.py
from datetime import timedelta
from typing import Annotated
from dotenv import load_dotenv
from fastapi import Depends, FastAPI, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import OAuth2PasswordRequestForm
from sqlalchemy.orm import Session
from pydantic import BaseModel

# Internal imports
from backend import auth, database, models, schemas, ai_service

load_dotenv()

app = FastAPI()

# Database & CORS
models.Base.metadata.create_all(bind=database.engine)

# If your frontend runs on localhost:3000 keep it, add more origins if needed.
origins = ["*"]
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- REQUEST MODELS (kept for convenience; also in schemas) ---
class TranslationRequest(BaseModel):
    text: str
    target_lang: str = "Urdu"

class ChatRequest(BaseModel):
    question: str

@app.get("/")
def root():
    return {"status": "Backend Running", "docs": "/docs"}

# --- AUTH ENDPOINTS ---
@app.post("/api/auth/signup", response_model=schemas.UserResponse)
def signup(user: schemas.UserCreate, db: Session = Depends(database.get_db)):
    db_user = db.query(models.User).filter(models.User.email == user.email).first()
    if db_user:
        raise HTTPException(status_code=400, detail="Email exists")

    hashed_password = auth.get_password_hash(user.password)
    new_user = models.User(
        email=user.email,
        hashed_password=hashed_password,
        experience_level=user.experience_level,
    )
    db.add(new_user)
    db.commit()
    db.refresh(new_user)
    return new_user

@app.post("/api/auth/login", response_model=schemas.Token)
def login(form_data: Annotated[OAuth2PasswordRequestForm, Depends()], db: Session = Depends(database.get_db)):
    user = db.query(models.User).filter(models.User.email == form_data.username).first()
    if not user or not auth.verify_password(form_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = auth.create_access_token(data={"sub": user.email}, expires_delta=timedelta(minutes=30))
    return {"access_token": token, "token_type": "bearer"}

# --- AI ENDPOINTS ---
@app.post("/api/translate")
def translate(request: TranslationRequest):
    return {"translated_text": ai_service.translate_text(request.text, request.target_lang)}

@app.post("/api/chat")
def chat(request: ChatRequest, user: models.User = Depends(auth.get_current_user)):
    """
    Protected chat endpoint.
    We wrap the ai_service call in try/except so unexpected errors return as HTTP 500
    and are logged server-side.
    """
    print(f"🤖 Chat Question from {user.email}: {request.question}")
    try:
        answer = ai_service.get_chat_response(request.question)
        return {"answer": answer}
    except Exception as e:
        # log the error server-side, but return a clean response to client
        print("❌ Error in ai_service.get_chat_response:", e)
        raise HTTPException(status_code=500, detail="Internal server error while generating answer")
