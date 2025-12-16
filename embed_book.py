#!/usr/bin/env python3
"""
Embed the entire book using Gemini API and store in Qdrant vector database.
Based on: https://ai.google.dev/gemini-api/docs/embeddings
"""

import os
import json
import time
from pathlib import Path
from typing import List, Dict, Any
from dotenv import load_dotenv
import requests
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

# Load environment variables
load_dotenv("src/.env")

# Configuration
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Gemini API settings (according to documentation)
GEMINI_API_ENDPOINT = "https://generativelanguage.googleapis.com/v1beta/models/text-embedding-004:embedContent"
EMBEDDING_MODEL = "text-embedding-004"
TASK_TYPE = "RETRIEVAL_DOCUMENT"  # Optimized for document retrieval
OUTPUT_DIMENSIONALITY = 768  # Using recommended dimension

# Collection name in Qdrant
COLLECTION_NAME = "physical_ai_book"

# Maximum tokens per embedding (from documentation)
MAX_TOKENS = 2048

# Docs directory
DOCS_DIR = Path("docs")


def chunk_text(text: str, max_length: int = 6000) -> List[str]:
    """
    Split text into chunks that won't exceed the token limit.
    Rough estimate: 1 token ≈ 3 characters for English text.
    Using 6000 chars to be safe under 2048 tokens.
    """
    chunks = []
    lines = text.split('\n')
    current_chunk = []
    current_length = 0

    for line in lines:
        line_length = len(line) + 1  # +1 for newline
        if current_length + line_length > max_length and current_chunk:
            chunks.append('\n'.join(current_chunk))
            current_chunk = [line]
            current_length = line_length
        else:
            current_chunk.append(line)
            current_length += line_length

    if current_chunk:
        chunks.append('\n'.join(current_chunk))

    return chunks


def generate_embedding(text: str) -> List[float]:
    """
    Generate embedding using Gemini API according to the documentation.
    """
    headers = {
        "Content-Type": "application/json",
        "x-goog-api-key": GEMINI_API_KEY
    }

    payload = {
        "model": f"models/{EMBEDDING_MODEL}",
        "content": {
            "parts": [{"text": text}]
        },
        "taskType": TASK_TYPE,
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


def read_markdown_files() -> List[Dict[str, Any]]:
    """
    Read all markdown files from the docs directory.
    """
    documents = []

    for md_file in DOCS_DIR.rglob("*.md"):
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Get relative path for metadata
            relative_path = md_file.relative_to(DOCS_DIR)

            documents.append({
                "path": str(relative_path),
                "full_path": str(md_file),
                "content": content,
                "module": relative_path.parts[0] if relative_path.parts else "root"
            })

        except Exception as e:
            print(f"Error reading {md_file}: {e}")
            continue

    return documents


def setup_qdrant_collection(client: QdrantClient):
    """
    Create or recreate the Qdrant collection.
    """
    try:
        # Delete collection if it exists
        try:
            client.delete_collection(collection_name=COLLECTION_NAME)
            print(f"Deleted existing collection: {COLLECTION_NAME}")
        except:
            pass

        # Create new collection
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=OUTPUT_DIMENSIONALITY,
                distance=Distance.COSINE
            )
        )
        print(f"Created new collection: {COLLECTION_NAME}")

    except Exception as e:
        print(f"Error setting up collection: {e}")
        raise


def embed_and_store():
    """
    Main function to embed all documents and store in Qdrant.
    """
    print("Starting book embedding process...")
    print(f"Using Gemini model: {EMBEDDING_MODEL}")
    print(f"Output dimensionality: {OUTPUT_DIMENSIONALITY}")
    print(f"Task type: {TASK_TYPE}")
    print()

    # Initialize Qdrant client
    print("Connecting to Qdrant...")
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY
    )

    # Setup collection
    setup_qdrant_collection(qdrant_client)

    # Read all markdown files
    print("\nReading markdown files...")
    documents = read_markdown_files()
    print(f"Found {len(documents)} documents")

    # Process each document
    points = []
    point_id = 0

    for idx, doc in enumerate(documents, 1):
        print(f"\n[{idx}/{len(documents)}] Processing: {doc['path']}")

        # Chunk the document if it's too long
        chunks = chunk_text(doc['content'])
        print(f"  Split into {len(chunks)} chunk(s)")

        for chunk_idx, chunk in enumerate(chunks):
            try:
                # Generate embedding
                print(f"  Generating embedding for chunk {chunk_idx + 1}...")
                embedding = generate_embedding(chunk)

                # Create point for Qdrant
                point = PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "file_path": doc['path'],
                        "full_path": doc['full_path'],
                        "module": doc['module'],
                        "chunk_index": chunk_idx,
                        "total_chunks": len(chunks),
                        "content": chunk[:500],  # Store first 500 chars for preview
                        "full_content": chunk
                    }
                )
                points.append(point)
                point_id += 1

                # Rate limiting - be nice to the API
                time.sleep(0.5)

            except Exception as e:
                print(f"  Error processing chunk {chunk_idx}: {e}")
                continue

        # Upload to Qdrant in batches of 10
        if len(points) >= 10:
            print(f"\n  Uploading batch of {len(points)} points to Qdrant...")
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                points=points
            )
            points = []

    # Upload remaining points
    if points:
        print(f"\nUploading final batch of {len(points)} points to Qdrant...")
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )

    # Get collection info
    collection_info = qdrant_client.get_collection(collection_name=COLLECTION_NAME)
    print(f"\n{'='*60}")
    print(f"Embedding complete!")
    print(f"Total points in collection: {collection_info.points_count}")
    print(f"Collection: {COLLECTION_NAME}")
    print(f"{'='*60}")


if __name__ == "__main__":
    embed_and_store()
