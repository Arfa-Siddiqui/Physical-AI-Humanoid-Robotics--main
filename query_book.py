#!/usr/bin/env python3
"""
Query the embedded book using semantic search.
Based on: https://ai.google.dev/gemini-api/docs/embeddings
"""

import os
import sys
from dotenv import load_dotenv
import requests
from qdrant_client import QdrantClient
from typing import List

# Fix Windows console encoding
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding='utf-8')

# Load environment variables
load_dotenv("src/.env")

# Configuration
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Gemini API settings
GEMINI_API_ENDPOINT = "https://generativelanguage.googleapis.com/v1beta/models/text-embedding-004:embedContent"
EMBEDDING_MODEL = "text-embedding-004"
TASK_TYPE = "RETRIEVAL_QUERY"  # Optimized for query retrieval
OUTPUT_DIMENSIONALITY = 768

# Collection name
COLLECTION_NAME = "physical_ai_book"


def generate_query_embedding(query: str) -> List[float]:
    """
    Generate embedding for query using Gemini API.
    Uses RETRIEVAL_QUERY task type for optimal query performance.
    """
    headers = {
        "Content-Type": "application/json",
        "x-goog-api-key": GEMINI_API_KEY
    }

    payload = {
        "model": f"models/{EMBEDDING_MODEL}",
        "content": {
            "parts": [{"text": query}]
        },
        "taskType": TASK_TYPE,  # Using RETRIEVAL_QUERY for queries
        "outputDimensionality": OUTPUT_DIMENSIONALITY
    }

    try:
        response = requests.post(
            GEMINI_API_ENDPOINT,
            headers=headers,
            json=payload,
            timeout=30
        )
        response.raise_for_status()

        result = response.json()
        embedding = result["embedding"]["values"]
        return embedding

    except requests.exceptions.RequestException as e:
        print(f"Error generating embedding: {e}")
        if hasattr(e, 'response') and e.response is not None:
            print(f"Response: {e.response.text}")
        raise


def search_book(query: str, limit: int = 5):
    """
    Search the embedded book using semantic search.
    """
    print(f"Searching for: '{query}'")
    print()

    # Initialize Qdrant client
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY
    )

    # Generate query embedding
    print("Generating query embedding...")
    query_embedding = generate_query_embedding(query)

    # Search in Qdrant
    print("Searching in vector database...")
    from qdrant_client.models import SearchRequest

    results = qdrant_client.query_points(
        collection_name=COLLECTION_NAME,
        query=query_embedding,
        limit=limit
    ).points

    # Display results
    print(f"\n{'='*80}")
    print(f"Found {len(results)} relevant results:")
    print(f"{'='*80}\n")

    for idx, result in enumerate(results, 1):
        score = result.score if hasattr(result, 'score') else 0
        payload = result.payload

        print(f"Result {idx} (Score: {score:.4f})")
        print(f"File: {payload.get('file_path', 'N/A')}")
        print(f"Module: {payload.get('module', 'N/A')}")
        print(f"Chunk: {payload.get('chunk_index', 0) + 1}/{payload.get('total_chunks', 1)}")
        print(f"\nContent preview:")
        print("-" * 80)
        print(payload.get('content', payload.get('full_content', 'N/A')[:500]))
        print("-" * 80)
        print()


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python query_book.py <your query>")
        print("\nExample queries:")
        print('  python query_book.py "How do I set up ROS2?"')
        print('  python query_book.py "What is URDF?"')
        print('  python query_book.py "Isaac Sim navigation"')
        sys.exit(1)

    query = " ".join(sys.argv[1:])
    search_book(query, limit=5)
