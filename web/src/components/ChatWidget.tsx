/**
 * ChatWidget Component
 *
 * Floating chat widget for interactive Q&A with the textbook.
 * Sends questions to the RAG-powered chatbot and displays answers
 * with source citations.
 *
 * Features:
 * - Floating widget (bottom-right)
 * - Message history
 * - Source citation display
 * - Context-aware questions from selected text (window.getSelection())
 * - Loading states and error handling
 */

import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWidget.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    filename: string;
    module: string;
    title: string;
    score: number;
  }>;
  timestamp: number;
}

interface ChatResponse {
  response: string;
  sources?: Array<{
    filename: string;
    module: string;
    title: string;
    score: number;
  }>;
}

export const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Auto-scroll to latest message
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Focus input when widget opens
  useEffect(() => {
    if (isOpen) {
      inputRef.current?.focus();
    }
  }, [isOpen]);

  /**
   * Extract selected text from the page for context-aware questions
   * Per spec FR-009: Inject window.getSelection() context
   */
  const getSelectionContext = (): string => {
    if (typeof window === 'undefined') return '';
    const selected = window.getSelection()?.toString() || '';
    return selected.substring(0, 200); // Limit context to 200 chars
  };

  /**
   * Send message to the chat endpoint
   * POST /chat with message and conversation history
   */
  const handleSendMessage = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!input.trim()) {
      return;
    }

    // Add user message to UI immediately
    const userMessage: Message = {
      id: `msg-${Date.now()}`,
      role: 'user',
      content: input,
      timestamp: Date.now(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);
    setError(null);

    try {
      // Prepare request with conversation history and selection context
      const selectionContext = getSelectionContext();
      const messageContent = selectionContext
        ? `${input}\n\n[Selected text context: "${selectionContext}"]`
        : input;

      // Convert history to format expected by API
      const history = messages.map((msg) => ({
        role: msg.role,
        content: msg.content,
      }));

      const response = await fetch('/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: messageContent,
          history,
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.statusText}`);
      }

      const data: ChatResponse = await response.json();

      // Add assistant response to messages
      const assistantMessage: Message = {
        id: `msg-${Date.now()}`,
        role: 'assistant',
        content: data.response,
        sources: data.sources || [],
        timestamp: Date.now(),
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Failed to get response';
      setError(errorMsg);

      // Add error message to chat
      const errorMessage: Message = {
        id: `msg-${Date.now()}`,
        role: 'assistant',
        content: `Sorry, I encountered an error: ${errorMsg}. Please try again.`,
        timestamp: Date.now(),
      };

      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Clear conversation history
   */
  const handleClearHistory = () => {
    setMessages([]);
    setError(null);
  };

  return (
    <div className={styles.chatWidgetContainer}>
      {/* Floating toggle button */}
      <button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
        title="Open chat assistant"
        aria-label="Chat assistant"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat widget window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.header}>
            <h3 className={styles.title}>Textbook Assistant</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          {/* Messages container */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && !error && (
              <div className={styles.emptyState}>
                <p>Ask me anything about the Physical AI & Humanoid Robotics textbook!</p>
                <p className={styles.hint}>Select text on the page for context-aware questions.</p>
              </div>
            )}

            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${styles[message.role]}`}
              >
                <div className={styles.messageContent}>
                  {message.content}
                </div>

                {/* Source citations for assistant messages */}
                {message.role === 'assistant' && message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <div className={styles.sourcesLabel}>📚 Sources:</div>
                    {message.sources.map((source, idx) => (
                      <div key={idx} className={styles.sourceItem}>
                        <span className={styles.sourceModule}>[{source.module}]</span>
                        {source.title && <span className={styles.sourceTitle}>{source.title}</span>}
                        <span className={styles.sourceScore}>({(source.score * 100).toFixed(0)}%)</span>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={styles.message + ' ' + styles.assistant}>
                <div className={styles.loadingIndicator}>
                  <span>●</span>
                  <span>●</span>
                  <span>●</span>
                </div>
              </div>
            )}

            {error && (
              <div className={styles.errorMessage}>
                <strong>⚠️ Error:</strong> {error}
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input area */}
          <div className={styles.inputArea}>
            <form onSubmit={handleSendMessage} className={styles.form}>
              <input
                ref={inputRef}
                type="text"
                placeholder="Ask a question..."
                value={input}
                onChange={(e) => setInput(e.target.value)}
                disabled={isLoading}
                className={styles.input}
              />
              <button
                type="submit"
                disabled={isLoading || !input.trim()}
                className={styles.sendButton}
              >
                Send
              </button>
            </form>

            {messages.length > 0 && (
              <button
                onClick={handleClearHistory}
                className={styles.clearButton}
                title="Clear conversation history"
              >
                Clear
              </button>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
