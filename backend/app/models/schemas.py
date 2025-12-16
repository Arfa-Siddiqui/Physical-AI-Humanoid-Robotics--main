from pydantic import BaseModel, Field
from typing import Optional, List


class ChatQueryRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=2000)
    chapter_id: Optional[str] = None
    session_id: Optional[str] = None


class SelectedTextRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=2000)
    selected_text: str = Field(..., min_length=1, max_length=10000)
    session_id: Optional[str] = None


class RetrievedChunk(BaseModel):
    content: str
    file_path: str
    module: str
    score: float


class ChatResponse(BaseModel):
    answer: str
    sources: List[RetrievedChunk] = []
    mode: str


class StreamChunk(BaseModel):
    content: str
    done: bool = False
