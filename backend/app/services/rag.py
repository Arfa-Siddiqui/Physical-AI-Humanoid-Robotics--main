from typing import AsyncGenerator, List, Tuple
from .qdrant import qdrant_service
from .gemini import gemini_service


BOOK_RAG_SYSTEM_PROMPT = """You are a concise AI assistant for the "Physical AI & Humanoid Robotics" book.

RULES:
1. Answer ONLY from CONTEXT below
2. Be brief and direct
3. If not in CONTEXT: "This is not covered in the book."
4. Quote key points directly

CONTEXT:
{context}

QUESTION: {question}

Answer:"""


SELECTED_TEXT_SYSTEM_PROMPT = """You are an AI assistant answering questions about a specific text selection.

STRICT RULES - YOU MUST FOLLOW THESE WITHOUT EXCEPTION:

1. Answer ONLY using information from the SELECTED TEXT below
2. DO NOT use any external knowledge, context, or information
3. DO NOT reference any other parts of the book or document
4. DO NOT make assumptions or inferences beyond what is explicitly stated in the SELECTED TEXT
5. If the answer is not in the SELECTED TEXT, you MUST respond with: "This is not present in the selected text."
6. Quote directly from the SELECTED TEXT when answering
7. Treat the SELECTED TEXT as the ONLY source of truth

SELECTED TEXT:
{selected_text}

USER QUESTION:
{question}

Remember: Answer ONLY from the SELECTED TEXT above. If not found, say "This is not present in the selected text."
"""


class RAGService:
    def __init__(self):
        self.qdrant = qdrant_service
        self.gemini = gemini_service

    async def answer_from_book(
        self,
        question: str,
        chapter_id: str = None
    ) -> AsyncGenerator[str, None]:
        """
        Answer question using book embeddings from Qdrant.

        Args:
            question: User's question
            chapter_id: Optional chapter/module filter

        Yields:
            Streaming response chunks
        """
        chunks = self.qdrant.search_book(
            query=question,
            limit=3,  # Reduced for faster responses
            module_filter=chapter_id
        )

        if not chunks:
            yield "This is not covered in the book."
            return

        context = self._build_context(chunks)

        prompt = BOOK_RAG_SYSTEM_PROMPT.format(
            context=context,
            question=question
        )

        system_instruction = "You are a helpful assistant that answers questions strictly based on provided context. Never use external knowledge."

        async for chunk in self.gemini.generate_stream(prompt, system_instruction):
            yield chunk

    async def answer_from_selected_text(
        self,
        question: str,
        selected_text: str
    ) -> AsyncGenerator[str, None]:
        """
        Answer question using ONLY the selected text.
        NO Qdrant retrieval allowed.

        Args:
            question: User's question
            selected_text: The text selected by user

        Yields:
            Streaming response chunks
        """
        prompt = SELECTED_TEXT_SYSTEM_PROMPT.format(
            selected_text=selected_text,
            question=question
        )

        system_instruction = "You are a helpful assistant that answers questions strictly based on the provided selected text. Never use external knowledge or context."

        async for chunk in self.gemini.generate_stream(prompt, system_instruction):
            yield chunk

    def get_relevant_chunks(
        self,
        question: str,
        chapter_id: str = None
    ) -> List[dict]:
        """
        Retrieve relevant chunks without generating answer.

        Args:
            question: User's question
            chapter_id: Optional chapter/module filter

        Returns:
            List of relevant chunks with metadata
        """
        return self.qdrant.search_book(
            query=question,
            limit=3,  # Reduced for faster responses
            module_filter=chapter_id
        )

    def _build_context(self, chunks: List[dict]) -> str:
        """Build context string from retrieved chunks."""
        context_parts = []

        for idx, chunk in enumerate(chunks, 1):
            file_path = chunk.get("file_path", "Unknown")
            content = chunk.get("content", "")
            score = chunk.get("score", 0.0)

            context_parts.append(
                f"[Source {idx}: {file_path} (relevance: {score:.2f})]\n{content}\n"
            )

        return "\n---\n".join(context_parts)


rag_service = RAGService()
