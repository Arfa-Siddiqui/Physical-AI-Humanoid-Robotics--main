import os
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue
import requests
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables
env_path = Path(__file__).parent.parent.parent.parent / "src" / ".env"
load_dotenv(env_path)

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
COLLECTION_NAME = "physical_ai_book"


class QdrantService:
    def __init__(self):
        self.client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY
        )
        self.collection_name = COLLECTION_NAME

    def generate_query_embedding(self, query: str) -> List[float]:
        """Generate embedding for query using Gemini API."""
        headers = {
            "Content-Type": "application/json",
            "x-goog-api-key": GEMINI_API_KEY
        }

        payload = {
            "model": "models/text-embedding-004",
            "content": {
                "parts": [{"text": query}]
            },
            "taskType": "RETRIEVAL_QUERY",
            "outputDimensionality": 768
        }

        response = requests.post(
            "https://generativelanguage.googleapis.com/v1beta/models/text-embedding-004:embedContent",
            headers=headers,
            json=payload,
            timeout=10  # Reduced from 30 to 10 seconds
        )
        response.raise_for_status()

        result = response.json()
        return result["embedding"]["values"]

    def search_book(
        self,
        query: str,
        limit: int = 3,  # Reduced from 5 to 3 for faster response
        module_filter: Optional[str] = None
    ) -> List[dict]:
        """
        Search embedded book content.

        Args:
            query: User's question
            limit: Number of results to return
            module_filter: Filter by module (e.g., "module-1-ros2")

        Returns:
            List of retrieved chunks with metadata
        """
        query_embedding = self.generate_query_embedding(query)

        search_params = {
            "collection_name": self.collection_name,
            "query": query_embedding,
            "limit": limit
        }

        if module_filter:
            search_params["query_filter"] = Filter(
                must=[
                    FieldCondition(
                        key="module",
                        match=MatchValue(value=module_filter)
                    )
                ]
            )

        results = self.client.query_points(**search_params).points

        chunks = []
        for result in results:
            chunks.append({
                "content": result.payload.get("full_content", ""),
                "file_path": result.payload.get("file_path", ""),
                "module": result.payload.get("module", ""),
                "score": result.score if hasattr(result, 'score') else 0.0,
                "chunk_index": result.payload.get("chunk_index", 0)
            })

        return chunks


qdrant_service = QdrantService()
