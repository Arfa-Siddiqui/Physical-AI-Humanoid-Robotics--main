from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os
from pathlib import Path

from .api import chat

# Load environment variables from src/.env
env_path = Path(__file__).parent.parent.parent / "src" / ".env"
load_dotenv(env_path)

app = FastAPI(
    title="Physical AI Book RAG API",
    description="RAG chatbot for Physical AI & Humanoid Robotics book",
    version="1.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://localhost:3001",
        "https://physical-ai-humanoid-robotics-heojh87p2-arfa-siddiquis-projects.vercel.app",
        "https://*.vercel.app",
    ],
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
            "chat_query": "/chat/query",
            "selected_text": "/chat/selected-text",
            "sources": "/chat/sources",
            "health": "/chat/health"
        }
    }


@app.get("/health")
async def health():
    return {"status": "ok"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
