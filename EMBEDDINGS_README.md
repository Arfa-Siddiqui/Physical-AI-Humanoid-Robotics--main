# Book Embeddings with Gemini API

This project includes semantic search capabilities for the Physical AI Humanoid Robotics book using Google's Gemini API for embeddings and Qdrant vector database for storage.

## Overview

The embedding system:
- Uses **Gemini API** (`text-embedding-004` model) to generate semantic embeddings
- Follows best practices from: https://ai.google.dev/gemini-api/docs/embeddings
- Stores embeddings in **Qdrant** vector database
- Enables semantic search across all book content
- Supports 51 document chunks across 31 markdown files

## Architecture

### Key Features
- **Model**: `text-embedding-004` (Gemini's latest embedding model)
- **Dimensions**: 768 (recommended for optimal performance)
- **Task Types**:
  - `RETRIEVAL_DOCUMENT`: For embedding book content
  - `RETRIEVAL_QUERY`: For processing search queries
- **Chunking**: Automatic text chunking to stay within 2,048 token limit
- **Vector Database**: Qdrant with cosine similarity search

## Files

### 1. `embed_book.py`
Main script to embed all book content and store in Qdrant.

**Features:**
- Reads all `.md` files from `docs/` directory
- Splits large documents into chunks (≤6000 chars ≈ 2048 tokens)
- Generates embeddings using Gemini API with optimal settings
- Uploads to Qdrant in batches
- Includes rate limiting and error handling

**Usage:**
```bash
python embed_book.py
```

**Output:**
- Creates/recreates `physical_ai_book` collection in Qdrant
- Stores embeddings with metadata (file path, module, chunk info, content)
- Shows progress for each document and chunk

### 2. `query_book.py`
Interactive semantic search script.

**Features:**
- Generates query embeddings optimized for search
- Performs cosine similarity search in Qdrant
- Returns top 5 most relevant results with scores
- Shows file path, module, and content preview

**Usage:**
```bash
# Query examples
python query_book.py "How to set up ROS2?"
python query_book.py "What is URDF?"
python query_book.py "Isaac Sim navigation"
python query_book.py "Vision-Language-Action models"
```

**Example Output:**
```
Searching for: 'How to set up ROS2?'

Generating query embedding...
Searching in vector database...

================================================================================
Found 5 relevant results:
================================================================================

Result 1 (Score: 0.7795)
File: module-1-ros2\prerequisites.md
Module: module-1-ros2
Chunk: 1/1

Content preview:
--------------------------------------------------------------------------------
# Prerequisites: Setting Up Your ROS 2 Environment
...
```

### 3. `requirements.txt`
Python dependencies:
```
requests>=2.31.0       # HTTP requests to Gemini API
python-dotenv>=1.0.0   # Environment variable management
qdrant-client>=1.7.0   # Qdrant vector database client
```

## Configuration

Environment variables in `src/.env`:

```bash
# Gemini API Key (from Google AI Studio)
GEMINI_API_KEY="your-api-key-here"

# Qdrant Vector Database
QDRANT_URL="your-qdrant-url"
QDRANT_API_KEY="your-qdrant-api-key"
```

## Setup Instructions

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Configure Environment
Ensure your `src/.env` file has valid credentials:
- Get Gemini API key from: https://makersuite.google.com/app/apikey
- Set up Qdrant (cloud or self-hosted): https://qdrant.tech/

### 3. Embed the Book
```bash
python embed_book.py
```

This will:
- Process all 31 markdown files
- Create 51 document chunks
- Generate embeddings for each chunk
- Store in Qdrant collection `physical_ai_book`

Expected time: ~2-3 minutes (with rate limiting)

### 4. Query the Book
```bash
python query_book.py "your search query"
```

## Embedding Statistics

- **Total Documents**: 31 markdown files
- **Total Chunks**: 51 (some documents split for length)
- **Collection Name**: `physical_ai_book`
- **Vector Dimensions**: 768
- **Distance Metric**: Cosine Similarity

### Coverage

**Modules Embedded:**
- `intro/` - 4 documents (8 chunks)
- `foundations/` - 3 documents (6 chunks)
- `module-1-ros2/` - 4 documents (7 chunks)
- `module-2-simulation/` - 4 documents (7 chunks)
- `module-3-isaac/` - 4 documents (7 chunks)
- `module-4-vla/` - 4 documents (6 chunks)
- `capstone/` - 3 documents (3 chunks)
- `code-examples/` - 5 documents (7 chunks)

## API Best Practices (from Gemini Docs)

1. **Task Type Optimization**:
   - Use `RETRIEVAL_DOCUMENT` for embedding documents
   - Use `RETRIEVAL_QUERY` for search queries

2. **Dimensionality**:
   - 768 dimensions provides best balance of performance/accuracy
   - Uses Matryoshka Representation Learning

3. **Token Limits**:
   - Maximum 2,048 tokens per embedding
   - Script automatically chunks longer documents

4. **Rate Limiting**:
   - 0.5 second delay between API calls
   - Batch uploads to Qdrant (every 10 points)

5. **Distance Metric**:
   - Cosine similarity recommended for text embeddings

## Querying Tips

### Effective Query Examples

**Good queries:**
- "How to set up ROS2?" (Score: 0.78)
- "URDF file structure for humanoids"
- "Isaac Sim installation requirements"
- "Nav2 configuration parameters"

**Query strategies:**
- Be specific and descriptive
- Use natural language questions
- Include domain-specific terms (ROS2, URDF, etc.)
- Longer queries often work better (more semantic information)

### Understanding Scores

- **0.9-1.0**: Extremely similar (rare, usually exact matches)
- **0.7-0.9**: Highly relevant
- **0.5-0.7**: Relevant but partial match
- **< 0.5**: Weak relevance

## Maintenance

### Re-embedding After Content Updates

When you update book content:
```bash
# Re-run the embedding script
python embed_book.py
```

This will:
- Delete the existing collection
- Re-create with updated content
- Preserve all configuration

### Adding New Content

New `.md` files in `docs/` are automatically included when you re-run `embed_book.py`.

## Troubleshooting

### Common Issues

**1. API Key Error**
```
Error generating embedding: 401 Unauthorized
```
**Solution**: Verify `GEMINI_API_KEY` in `src/.env`

**2. Qdrant Connection Error**
```
Error setting up collection: Connection refused
```
**Solution**: Check `QDRANT_URL` and `QDRANT_API_KEY` in `src/.env`

**3. Token Limit Exceeded**
```
Error: Input too long
```
**Solution**: The script should auto-chunk, but you can reduce `max_length` in `chunk_text()`

**4. Unicode Errors (Windows)**
Fixed in script with `sys.stdout.reconfigure(encoding='utf-8')`

## Advanced Usage

### Programmatic Access

```python
from qdrant_client import QdrantClient
from query_book import generate_query_embedding

# Initialize client
client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Search
query_vector = generate_query_embedding("your query")
results = client.query_points(
    collection_name="physical_ai_book",
    query=query_vector,
    limit=10
).points

for result in results:
    print(f"Score: {result.score}")
    print(f"Content: {result.payload['full_content']}")
```

### Custom Task Types

Modify `TASK_TYPE` in scripts for different use cases:
- `SEMANTIC_SIMILARITY` - General similarity
- `CLASSIFICATION` - Document categorization
- `CLUSTERING` - Group similar documents
- `QUESTION_ANSWERING` - Q&A optimization

## References

- **Gemini Embeddings Docs**: https://ai.google.dev/gemini-api/docs/embeddings
- **Qdrant Documentation**: https://qdrant.tech/documentation/
- **Google AI Studio**: https://makersuite.google.com/

## License

Same as main project.
