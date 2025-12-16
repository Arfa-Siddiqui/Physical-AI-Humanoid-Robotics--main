import React, { useState, useEffect, useRef } from 'react';
import { useTextSelection } from '../hooks/useTextSelection';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  mode?: 'book' | 'selected-text';
}

interface BookChatProps {
  chapterId?: string;
  apiBaseUrl?: string;
}

export const BookChat: React.FC<BookChatProps> = ({
  chapterId,
  apiBaseUrl = 'http://localhost:8000',
}) => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [mode, setMode] = useState<'book' | 'selected-text'>('book');
  const [isOpen, setIsOpen] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const { text: selectedText, hasSelection } = useTextSelection();

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSendMessage = async () => {
    if (!input.trim()) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: input,
      mode,
    };

    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: '',
        mode,
      };

      setMessages((prev) => [...prev, assistantMessage]);

      const endpoint =
        mode === 'selected-text' ? '/chat/selected-text' : '/chat/query';

      const body =
        mode === 'selected-text'
          ? {
              question: input,
              selected_text: selectedText,
            }
          : {
              question: input,
              chapter_id: chapterId,
            };

      const response = await fetch(`${apiBaseUrl}${endpoint}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(body),
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const reader = response.body?.getReader();
      const decoder = new TextDecoder();

      if (!reader) {
        throw new Error('No reader available');
      }

      let accumulatedContent = '';

      while (true) {
        const { done, value } = await reader.read();

        if (done) break;

        const chunk = decoder.decode(value, { stream: true });
        const lines = chunk.split('\n');

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const data = line.substring(6);
            try {
              const parsed = JSON.parse(data);
              if (parsed.content) {
                accumulatedContent += parsed.content;
                setMessages((prev) => {
                  const newMessages = [...prev];
                  const lastMessage = newMessages[newMessages.length - 1];
                  if (lastMessage.role === 'assistant') {
                    lastMessage.content = accumulatedContent;
                  }
                  return newMessages;
                });
              }
              if (parsed.done) {
                break;
              }
            } catch (e) {
              console.error('Failed to parse SSE data:', e);
            }
          }
        }
      }
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages((prev) => [
        ...prev,
        {
          id: Date.now().toString(),
          role: 'assistant',
          content: 'Sorry, there was an error processing your request.',
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const toggleMode = () => {
    if (mode === 'book') {
      if (hasSelection) {
        setMode('selected-text');
      } else {
        alert('Please select some text first to use selected-text mode.');
      }
    } else {
      setMode('book');
    }
  };

  // Show popup button when chat is closed
  if (!isOpen) {
    return (
      <button
        onClick={() => setIsOpen(true)}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#007bff',
          color: '#fff',
          border: 'none',
          fontSize: '28px',
          cursor: 'pointer',
          boxShadow: '0 4px 12px rgba(0,0,0,0.3)',
          zIndex: 1000,
          transition: 'transform 0.2s, box-shadow 0.2s',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.transform = 'scale(1.1)';
          e.currentTarget.style.boxShadow = '0 6px 16px rgba(0,0,0,0.4)';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = 'scale(1)';
          e.currentTarget.style.boxShadow = '0 4px 12px rgba(0,0,0,0.3)';
        }}
        title="Open Book Assistant"
      >
        💬
      </button>
    );
  }

  return (
    <div
      style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        width: '400px',
        height: '600px',
        border: '1px solid #ddd',
        borderRadius: '12px',
        backgroundColor: '#fff',
        boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
        display: 'flex',
        flexDirection: 'column',
        zIndex: 1000,
      }}
    >
      {/* Header */}
      <div
        style={{
          padding: '16px',
          borderBottom: '1px solid #eee',
          backgroundColor: '#007bff',
          color: '#fff',
          borderTopLeftRadius: '12px',
          borderTopRightRadius: '12px',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
        }}
      >
        <h3 style={{ margin: 0, fontSize: '16px', fontWeight: 600 }}>
          📚 Book Assistant
        </h3>
        <button
          onClick={() => setIsOpen(false)}
          style={{
            background: 'none',
            border: 'none',
            color: '#fff',
            fontSize: '24px',
            cursor: 'pointer',
            padding: '0',
            width: '30px',
            height: '30px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
          }}
          title="Close"
        >
          ✕
        </button>
      </div>

      {/* Mode Toggle */}
      <div
        style={{
          padding: '12px 16px',
          backgroundColor: '#f8f9fa',
          borderBottom: '1px solid #eee',
        }}
      >
        <div style={{ display: 'flex', gap: '8px' }}>
          <button
            onClick={toggleMode}
            style={{
              padding: '6px 12px',
              fontSize: '12px',
              border: '1px solid #ddd',
              borderRadius: '6px',
              backgroundColor: mode === 'book' ? '#007bff' : '#fff',
              color: mode === 'book' ? '#fff' : '#333',
              cursor: 'pointer',
            }}
          >
            Full Book
          </button>
          <button
            onClick={toggleMode}
            style={{
              padding: '6px 12px',
              fontSize: '12px',
              border: '1px solid #ddd',
              borderRadius: '6px',
              backgroundColor: mode === 'selected-text' ? '#007bff' : '#fff',
              color: mode === 'selected-text' ? '#fff' : '#333',
              cursor: 'pointer',
              opacity: hasSelection ? 1 : 0.5,
            }}
            disabled={!hasSelection && mode === 'book'}
          >
            Selected Text
          </button>
        </div>
        {mode === 'selected-text' && hasSelection && (
          <div
            style={{
              marginTop: '8px',
              padding: '8px',
              backgroundColor: '#e7f3ff',
              borderRadius: '4px',
              fontSize: '12px',
              color: '#0066cc',
            }}
          >
            ✓ Text selected ({selectedText.length} chars)
          </div>
        )}
      </div>

      {/* Messages */}
      <div
        style={{
          flex: 1,
          overflowY: 'auto',
          padding: '16px',
          display: 'flex',
          flexDirection: 'column',
          gap: '12px',
        }}
      >
        {messages.length === 0 && (
          <div
            style={{
              textAlign: 'center',
              color: '#888',
              marginTop: '20px',
              fontSize: '14px',
            }}
          >
            <p>👋 Ask me anything about the book!</p>
            <p style={{ fontSize: '12px', marginTop: '8px' }}>
              Switch to "Selected Text" mode to ask about specific selections.
            </p>
          </div>
        )}

        {messages.map((msg) => (
          <div
            key={msg.id}
            style={{
              alignSelf: msg.role === 'user' ? 'flex-end' : 'flex-start',
              maxWidth: '80%',
            }}
          >
            <div
              style={{
                padding: '10px 14px',
                borderRadius: '12px',
                backgroundColor: msg.role === 'user' ? '#007bff' : '#f1f3f4',
                color: msg.role === 'user' ? '#fff' : '#333',
                fontSize: '14px',
                lineHeight: '1.5',
              }}
            >
              {msg.content}
            </div>
            {msg.mode && (
              <div
                style={{
                  fontSize: '10px',
                  color: '#888',
                  marginTop: '4px',
                  textAlign: msg.role === 'user' ? 'right' : 'left',
                }}
              >
                {msg.mode === 'selected-text' ? '📝 Selected' : '📚 Book'}
              </div>
            )}
          </div>
        ))}

        {isLoading && (
          <div
            style={{
              alignSelf: 'flex-start',
              padding: '10px 14px',
              borderRadius: '12px',
              backgroundColor: '#f1f3f4',
              color: '#888',
              fontSize: '14px',
            }}
          >
            Thinking...
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <div
        style={{
          padding: '16px',
          borderTop: '1px solid #eee',
          backgroundColor: '#f8f9fa',
          borderBottomLeftRadius: '12px',
          borderBottomRightRadius: '12px',
        }}
      >
        <div style={{ display: 'flex', gap: '8px' }}>
          <textarea
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder={
              mode === 'selected-text'
                ? 'Ask about selected text...'
                : 'Ask a question...'
            }
            style={{
              flex: 1,
              padding: '10px',
              border: '1px solid #ddd',
              borderRadius: '8px',
              fontSize: '14px',
              resize: 'none',
              minHeight: '60px',
              fontFamily: 'inherit',
            }}
            disabled={isLoading}
          />
          <button
            onClick={handleSendMessage}
            disabled={isLoading || !input.trim()}
            style={{
              padding: '10px 20px',
              backgroundColor: '#007bff',
              color: '#fff',
              border: 'none',
              borderRadius: '8px',
              cursor: isLoading || !input.trim() ? 'not-allowed' : 'pointer',
              fontSize: '14px',
              fontWeight: 600,
              opacity: isLoading || !input.trim() ? 0.5 : 1,
            }}
          >
            Send
          </button>
        </div>
      </div>
    </div>
  );
};

export default BookChat;
