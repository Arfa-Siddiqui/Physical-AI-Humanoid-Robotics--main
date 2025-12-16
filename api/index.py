from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from mangum import Mangum
import sys
from pathlib import Path

# Add backend to path
backend_path = Path(__file__).parent.parent / "backend"
sys.path.insert(0, str(backend_path))

from app.api import chat

app = FastAPI(
    title="Physical AI Book RAG API",
    description="RAG chatbot for Physical AI & Humanoid Robotics book",
    version="1.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(chat.router)

@app.get("/")
async def root():
    return {
        "service": "Physical AI Book RAG API",
        "status": "running",
        "endpoints": {
            "chat_query": "/api/chat/query",
            "selected_text": "/api/chat/selected-text",
            "sources": "/api/chat/sources",
            "health": "/api/chat/health"
        }
    }

@app.get("/health")
async def health():
    return {"status": "ok"}

handler = Mangum(app)
