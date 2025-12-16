---
sidebar_position: 100
---

# AI Assistant

Try the AI-powered book assistant! Ask questions about any topic covered in the book.

## Features

### 📚 Full Book Mode
Ask questions and get answers based on the entire book content. The AI retrieves relevant sections and provides grounded answers.

**Example questions:**
- "How do I install ROS2?"
- "What is URDF?"
- "Explain Isaac Sim navigation"
- "What are Vision-Language-Action models?"

### 📝 Selected Text Mode
Select any text on this page (or any other page) and ask specific questions about it. The AI will answer ONLY from your selection.

**Try it:**
1. Select the text below
2. Switch to "Selected Text" mode in the chat
3. Ask a question about it

> ROS 2 (Robot Operating System 2) is a set of software libraries and tools for building robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS 2 is designed to be a flexible framework for writing robot software, with a focus on scalability, real-time performance, and multi-robot systems.

**Example questions for selection:**
- "What is ROS 2?"
- "What features does it provide?"
- "What is the focus of ROS 2?"

## How It Works

### Full Book Mode
1. Your question is converted to an embedding
2. Relevant book sections are retrieved from the vector database
3. The AI reads these sections and answers your question
4. If the answer isn't in the book, it says so explicitly

### Selected Text Mode
1. You select text on the page
2. Your question is sent with ONLY that text
3. The AI answers based solely on your selection
4. No other book content is consulted

## Try It Now

The chat widget should appear in the bottom-right corner. If you don't see it, make sure:
- The backend server is running (`http://localhost:8000`)
- JavaScript is enabled in your browser
- You've integrated the BookChat component

## Tips

1. **Be specific** - More detailed questions get better answers
2. **Use context** - Mention the module or topic if you know it
3. **Try selections** - Selected text mode is great for understanding specific passages
4. **Check sources** - The AI shows which files it used to answer

## Limitations

- The AI only knows what's in the book (by design)
- It won't provide external information or opinions
- For code examples, check the actual code-examples directory
- Long responses may take a few seconds to stream

## Questions?

If you encounter issues:
1. Check the RAG_INTEGRATION.md guide
2. Verify the backend is running
3. Look at browser console for errors
4. Test the API with `python backend/test_api.py`
