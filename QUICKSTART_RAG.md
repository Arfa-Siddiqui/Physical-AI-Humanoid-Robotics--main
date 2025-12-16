# RAG Chatbot Quick Start

Get the AI assistant running in 5 minutes.

## Prerequisites

- Python 3.11+
- Node.js 18+
- Existing Qdrant embeddings (already done)
- Gemini API key in `src/.env`

## Step 1: Start Backend (2 minutes)

```bash
# Install dependencies
cd backend
pip install -r requirements.txt

# Start server
python run.py
```

Server starts on: `http://localhost:8000`

**Verify:**
```bash
curl http://localhost:8000/health
# Should return: {"status": "ok"}
```

## Step 2: Test Backend (1 minute)

```bash
cd backend
python test_api.py
```

Expected output:
```
============================================================
RAG API Test Suite
============================================================
Testing health endpoint...
✓ Health check passed
Testing chat health endpoint...
✓ Chat health check passed
...
============================================================
Test Results
============================================================
Passed: 6/6
✓ All tests passed!
```

## Step 3: Add Chat to Frontend (2 minutes)

### Option A: Test in Browser Console

Open `http://localhost:3000` and paste in console:

```javascript
// Test full book query
fetch('http://localhost:8000/chat/query', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ question: 'What is ROS2?' })
}).then(r => r.text()).then(console.log)
```

### Option B: Add to Specific Page

Edit any `.md` or `.mdx` file in `docs/`:

```mdx
---
title: Your Page
---

import BookChat from '@site/src/components/BookChat';

# Your Content

<BookChat chapterId="module-1-ros2" />
```

### Option C: Global Integration

Edit `src/theme/Root.js` (create if doesn't exist):

```jsx
import React from 'react';
import BookChat from '../components/BookChat';

export default function Root({children}) {
  return (
    <>
      {children}
      <BookChat />
    </>
  );
}
```

## Step 4: Use the Assistant

### Full Book Mode

1. Open chat widget (bottom-right)
2. Ask: "How do I install ROS2?"
3. Get streaming answer from book content

### Selected Text Mode

1. Select any text on the page
2. Click "Selected Text" button
3. Ask: "Explain this"
4. Get answer from ONLY selected text

## Common Issues

### Backend won't start

**Error:** `ModuleNotFoundError`
```bash
# Fix: Install dependencies
cd backend
pip install -r requirements.txt
```

**Error:** `Qdrant connection failed`
```bash
# Fix: Check environment variables
cat ../src/.env
# Verify QDRANT_URL and QDRANT_API_KEY
```

**Error:** `Gemini API error`
```bash
# Fix: Check API key
cat ../src/.env
# Verify GEMINI_API_KEY
```

### Frontend issues

**Chat doesn't appear:**
- Ensure backend is running: `curl http://localhost:8000/health`
- Check browser console for errors
- Verify component import path

**Streaming doesn't work:**
- Check CORS in backend `main.py`
- Verify fetch URL uses correct port
- Check network tab in browser DevTools

### CORS errors

Add your frontend URL to backend `main.py`:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://localhost:3001",
        "https://your-domain.com"  # Add your domain
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Testing Different Queries

### ROS2 Questions
```
"How do I set up ROS2?"
"What is a ROS2 node?"
"Explain ROS2 topics"
```

### URDF Questions
```
"What is URDF?"
"How do I create a URDF file?"
"Explain URDF links and joints"
```

### Isaac Sim Questions
```
"How do I install Isaac Sim?"
"What is Isaac ROS?"
"Explain Nav2 integration"
```

### VLA Questions
```
"What are Vision-Language-Action models?"
"How does voice control work?"
"Explain LLM planning"
```

## Next Steps

1. **Add to all pages:** Use Root.js for global chat
2. **Customize styling:** Edit BookChat.tsx styles
3. **Add analytics:** Track queries in backend
4. **Deploy:** See RAG_INTEGRATION.md deployment section

## Architecture Recap

```
┌─────────────┐
│   Browser   │
│  (Docusaurus)│
└──────┬──────┘
       │ HTTP POST
       ▼
┌─────────────┐
│   FastAPI   │
│   Backend   │
└──────┬──────┘
       │
       ├─────────────┐
       ▼             ▼
┌──────────┐   ┌──────────┐
│  Qdrant  │   │  Gemini  │
│ (Vectors)│   │   (LLM)  │
└──────────┘   └──────────┘
```

## Performance

- **Response time:** 1-3 seconds
- **Streaming:** Starts in <500ms
- **Concurrent users:** 10+ (FastAPI async)
- **Rate limits:** Gemini API limits apply

## Full Documentation

- **Integration Guide:** `RAG_INTEGRATION.md`
- **Embeddings Setup:** `EMBEDDINGS_README.md`
- **API Testing:** `backend/test_api.py`
- **Component Props:** See RAG_INTEGRATION.md

## Support

Issues? Check:
1. Backend logs: `python run.py` output
2. Frontend logs: Browser console
3. API tests: `python backend/test_api.py`
4. Health check: `curl http://localhost:8000/health`

## Success Criteria

✅ Backend starts without errors
✅ All 6 API tests pass
✅ Chat widget appears in frontend
✅ Full book queries return relevant answers
✅ Selected text mode works independently
✅ Streaming responses display correctly
✅ No hallucinations (strict grounding works)

Done! You now have a production-ready RAG chatbot integrated with your book.
