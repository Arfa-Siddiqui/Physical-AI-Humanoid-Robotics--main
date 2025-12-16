# RAG Chatbot Integration Guide

Production-ready RAG chatbot integrated with existing Qdrant embeddings.

## Architecture

```
User Question → FastAPI Backend → Qdrant (Retrieval) → Gemini LLM → Streaming Response
                                ↓
                         Selected Text Mode (NO Qdrant)
```

## Components Created

### Backend (FastAPI)

```
backend/
├── app/
│   ├── main.py              # FastAPI app
│   ├── api/
│   │   └── chat.py          # Chat endpoints
│   ├── services/
│   │   ├── qdrant.py        # Qdrant retrieval
│   │   ├── gemini.py        # Gemini LLM integration
│   │   └── rag.py           # RAG logic with strict grounding
│   └── models/
│       └── schemas.py       # Pydantic models
├── requirements.txt
└── run.py
```

### Frontend (React/TypeScript)

```
src/
├── components/
│   └── BookChat.tsx         # Main chat component
└── hooks/
    └── useTextSelection.ts  # Text selection hook
```

## Setup Instructions

### 1. Install Backend Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Verify Environment Variables

Ensure `src/.env` contains:

```bash
GEMINI_API_KEY=your-key-here
QDRANT_URL=your-qdrant-url
QDRANT_API_KEY=your-qdrant-key
```

### 3. Start Backend Server

```bash
cd backend
python run.py
```

Backend runs on: `http://localhost:8000`

**Endpoints:**
- `POST /chat/query` - Full book RAG mode
- `POST /chat/selected-text` - Selected text only mode
- `POST /chat/sources` - Get sources without answer
- `GET /chat/health` - Health check

### 4. Test Backend

```bash
# Test full book RAG
curl -X POST http://localhost:8000/chat/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS2?"}'

# Test selected text mode
curl -X POST http://localhost:8000/chat/selected-text \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain this",
    "selected_text": "ROS2 is a robotics middleware..."
  }'
```

### 5. Integrate Frontend

#### Option A: Add to Docusaurus Page

Create a new file: `docs/chat-demo.md`

```mdx
---
title: AI Assistant Demo
---

import BookChat from '@site/src/components/BookChat';

# AI Assistant Demo

Try asking questions about the book!

<BookChat chapterId="module-1-ros2" />
```

#### Option B: Global Integration

Edit `docusaurus.config.js`:

```javascript
module.exports = {
  // ... other config
  themeConfig: {
    // Add custom script
  },
  clientModules: [
    require.resolve('./src/components/BookChatGlobal.tsx'),
  ],
};
```

Create `src/components/BookChatGlobal.tsx`:

```tsx
import React from 'react';
import BookChat from './BookChat';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

if (ExecutionEnvironment.canUseDOM) {
  const root = document.createElement('div');
  document.body.appendChild(root);

  import('react-dom/client').then(({ createRoot }) => {
    createRoot(root).render(<BookChat />);
  });
}

export default null;
```

## Two Modes Explained

### Mode 1: Full Book RAG

**Usage:** Default mode for general questions

**Flow:**
1. User asks question
2. Backend generates query embedding (Gemini)
3. Qdrant retrieves top 5 relevant chunks
4. Context built from retrieved chunks
5. Gemini answers ONLY from context
6. Streams response to frontend

**Grounding:** Strict system prompt enforces "This is not covered in the book." if answer not found.

**Example:**
```
Q: "How do I install ROS2?"
A: [Retrieves from module-1-ros2/prerequisites.md]
   "To install ROS2 Humble on Ubuntu 22.04..."
```

### Mode 2: Selected Text Only

**Usage:** When user selects text on page and asks specific question

**Flow:**
1. User selects text on page
2. Clicks "Selected Text" mode
3. Asks question about selection
4. Backend receives ONLY selected text
5. Gemini answers ONLY from selected text
6. NO Qdrant retrieval
7. Streams response to frontend

**Grounding:** Ultra-strict prompt enforces "This is not present in the selected text." if answer not found.

**Example:**
```
Selected: "URDF stands for Unified Robot Description Format..."
Q: "What does URDF stand for?"
A: "Based on the selected text, URDF stands for Unified Robot Description Format."
```

## Strict Grounding Implementation

### Book RAG System Prompt

```
STRICT RULES:
1. Answer ONLY from provided CONTEXT
2. NO external knowledge
3. NO assumptions beyond CONTEXT
4. If not in CONTEXT → "This is not covered in the book."
5. Quote directly when possible
```

### Selected Text System Prompt

```
STRICT RULES:
1. Answer ONLY from SELECTED TEXT
2. NO external knowledge or book context
3. NO references to other content
4. If not in SELECTED TEXT → "This is not present in the selected text."
5. Quote directly from SELECTED TEXT
```

## API Reference

### POST /chat/query

**Request:**
```json
{
  "question": "What is ROS2?",
  "chapter_id": "module-1-ros2",  // optional filter
  "session_id": "uuid"            // optional
}
```

**Response:** Server-Sent Events (SSE)
```
data: {"content": "ROS2 is", "done": false}
data: {"content": " a robotics", "done": false}
data: {"content": " middleware", "done": false}
data: {"content": "", "done": true}
```

### POST /chat/selected-text

**Request:**
```json
{
  "question": "What does this mean?",
  "selected_text": "The text user selected...",
  "session_id": "uuid"  // optional
}
```

**Response:** Same SSE format

### POST /chat/sources

**Request:**
```json
{
  "question": "What is ROS2?",
  "chapter_id": "module-1-ros2"
}
```

**Response:**
```json
{
  "answer": "",
  "sources": [
    {
      "content": "First 500 chars...",
      "file_path": "module-1-ros2/01-ros2-architecture.md",
      "module": "module-1-ros2",
      "score": 0.85
    }
  ],
  "mode": "sources_only"
}
```

## Frontend Component Props

### BookChat

```tsx
interface BookChatProps {
  chapterId?: string;      // Filter by chapter/module
  apiBaseUrl?: string;     // Default: http://localhost:8000
}

// Usage
<BookChat chapterId="module-1-ros2" />
<BookChat apiBaseUrl="https://your-api.com" />
```

### useTextSelection Hook

```tsx
const { text, hasSelection } = useTextSelection();

// Returns:
// text: string - Currently selected text
// hasSelection: boolean - Whether text is selected
```

## Testing Checklist

### Backend Tests

- [ ] Server starts without errors: `python backend/run.py`
- [ ] Health check responds: `curl http://localhost:8000/health`
- [ ] Chat health responds: `curl http://localhost:8000/chat/health`
- [ ] Full book query works with streaming
- [ ] Selected text query works with streaming
- [ ] Invalid requests return 400 errors
- [ ] Qdrant connection is successful
- [ ] Gemini API responses are streaming
- [ ] Chapter filtering works correctly

### Frontend Tests

- [ ] BookChat component renders
- [ ] Mode toggle works (Book ↔ Selected Text)
- [ ] Text selection is detected
- [ ] Selected text mode activates with selection
- [ ] Messages display correctly
- [ ] Streaming responses render in real-time
- [ ] Input field accepts text
- [ ] Send button triggers request
- [ ] Loading state shows "Thinking..."
- [ ] Errors display properly

### Integration Tests

- [ ] Full book RAG returns relevant answers
- [ ] Answers are grounded (no hallucinations)
- [ ] "Not covered" response when appropriate
- [ ] Selected text mode uses ONLY selected text
- [ ] Selected text mode never queries Qdrant
- [ ] Chapter filtering limits results correctly
- [ ] Streaming works end-to-end
- [ ] Multiple queries in succession work
- [ ] Long responses stream properly

## Production Deployment

### Backend

**Option 1: Railway/Render**
```bash
# Add Procfile
web: uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

**Option 2: Docker**
```dockerfile
FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY . .
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**Environment Variables:**
- Set `GEMINI_API_KEY`
- Set `QDRANT_URL`
- Set `QDRANT_API_KEY`
- Update CORS origins in `main.py`

### Frontend

Update `apiBaseUrl` in component:

```tsx
<BookChat
  apiBaseUrl="https://your-backend-api.com"
  chapterId={currentChapter}
/>
```

## Troubleshooting

### Backend Issues

**Error: "Qdrant connection failed"**
- Verify `QDRANT_URL` and `QDRANT_API_KEY`
- Check Qdrant cloud status
- Test connection: `curl $QDRANT_URL/collections`

**Error: "Gemini API error"**
- Verify `GEMINI_API_KEY`
- Check quota/billing at Google AI Studio
- Test directly: use `query_book.py`

**Error: "Module not found"**
- Ensure all `__init__.py` files exist
- Run from `backend/` directory
- Check Python path

### Frontend Issues

**Chat not appearing**
- Check browser console for errors
- Verify backend is running
- Check CORS settings in backend
- Verify component import path

**Streaming not working**
- Check browser supports SSE
- Verify Content-Type is text/event-stream
- Check network tab for response
- Ensure no proxy blocking

**Text selection not detected**
- Check hook is imported correctly
- Verify event listeners are attached
- Test in different browser
- Check for conflicting event handlers

## Performance Optimization

### Backend

1. **Caching:** Add Redis for frequent queries
2. **Batch Processing:** Group Qdrant queries
3. **Connection Pooling:** Reuse HTTP connections
4. **Rate Limiting:** Protect API endpoints

### Frontend

1. **Debouncing:** Delay text selection updates
2. **Message Pagination:** Limit displayed messages
3. **Lazy Loading:** Load chat on demand
4. **Code Splitting:** Separate chat bundle

## Security Considerations

1. **API Key Protection:** Never expose keys in frontend
2. **Rate Limiting:** Implement on backend
3. **Input Validation:** Sanitize all inputs
4. **CORS:** Restrict to known origins
5. **Content Filtering:** Validate selected text length
6. **Session Management:** Add session limits

## Next Steps

1. **Add Memory:** Integrate Neon Postgres for conversation history
2. **Add Analytics:** Track query patterns
3. **Add Feedback:** Allow users to rate answers
4. **Add Citations:** Show exact source locations
5. **Add Export:** Let users save conversations
6. **Add Voice:** Integrate speech-to-text

## Support

- Backend logs: Check `uvicorn` output
- Frontend logs: Browser console
- API testing: Use Postman/curl
- Embeddings: Run `query_book.py` to test Qdrant

## License

Same as main project.
