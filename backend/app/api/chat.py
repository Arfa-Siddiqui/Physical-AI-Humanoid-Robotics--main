from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from typing import AsyncGenerator
import json
from ..models.schemas import (
    ChatQueryRequest,
    SelectedTextRequest,
    ChatResponse,
    RetrievedChunk
)
from ..services.rag import rag_service


router = APIRouter(prefix="/chat", tags=["chat"])


async def stream_response(generator: AsyncGenerator[str, None]) -> AsyncGenerator[bytes, None]:
    """Convert async generator to SSE format."""
    try:
        async for chunk in generator:
            data = json.dumps({"content": chunk, "done": False})
            yield f"data: {data}\n\n".encode('utf-8')

        final_data = json.dumps({"content": "", "done": True})
        yield f"data: {final_data}\n\n".encode('utf-8')

    except Exception as e:
        error_data = json.dumps({"error": str(e), "done": True})
        yield f"data: {error_data}\n\n".encode('utf-8')


@router.post("/query")
async def query_book(request: ChatQueryRequest):
    """
    Answer questions using full book RAG with Qdrant retrieval.

    Mode: FULL BOOK RAG
    - Retrieves relevant chunks from Qdrant
    - Optionally filters by chapter_id
    - Streams response from Gemini
    - Enforces strict grounding to retrieved context
    """
    try:
        if not request.question.strip():
            raise HTTPException(status_code=400, detail="Question cannot be empty")

        response_generator = rag_service.answer_from_book(
            question=request.question,
            chapter_id=request.chapter_id
        )

        return StreamingResponse(
            stream_response(response_generator),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no"
            }
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/selected-text")
async def query_selected_text(request: SelectedTextRequest):
    """
    Answer questions using ONLY the selected text.

    Mode: SELECTED TEXT ONLY
    - NO Qdrant retrieval
    - NO book context
    - Answers ONLY from selected_text
    - Enforces strict grounding to selected text
    """
    try:
        if not request.question.strip():
            raise HTTPException(status_code=400, detail="Question cannot be empty")

        if not request.selected_text.strip():
            raise HTTPException(status_code=400, detail="Selected text cannot be empty")

        response_generator = rag_service.answer_from_selected_text(
            question=request.question,
            selected_text=request.selected_text
        )

        return StreamingResponse(
            stream_response(response_generator),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no"
            }
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/sources")
async def get_sources(request: ChatQueryRequest) -> ChatResponse:
    """
    Get relevant sources without generating answer.

    Returns:
        List of relevant chunks with metadata
    """
    try:
        chunks = rag_service.get_relevant_chunks(
            question=request.question,
            chapter_id=request.chapter_id
        )

        sources = [
            RetrievedChunk(
                content=chunk["content"][:500],
                file_path=chunk["file_path"],
                module=chunk["module"],
                score=chunk["score"]
            )
            for chunk in chunks
        ]

        return ChatResponse(
            answer="",
            sources=sources,
            mode="sources_only"
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "ok", "service": "chat"}
