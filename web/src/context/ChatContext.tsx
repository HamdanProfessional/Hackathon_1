/**
 * ChatContext - Global Chat State Management
 *
 * Manages chat widget state across the entire application:
 * - Chat open/close state
 * - Active text contexts (selected text snippets)
 * - Context management (add/remove)
 */

import React, { createContext, useContext, useState, ReactNode } from 'react';

interface ChatContextType {
  isChatOpen: boolean;
  setIsChatOpen: (open: boolean) => void;
  activeContexts: string[];
  addContext: (text: string) => void;
  removeContext: (index: number) => void;
  clearContexts: () => void;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

interface ChatProviderProps {
  children: ReactNode;
}

export function ChatProvider({ children }: ChatProviderProps) {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [activeContexts, setActiveContexts] = useState<string[]>([]);

  const addContext = (text: string) => {
    // Trim and limit context length
    const trimmedText = text.trim().substring(0, 500);

    if (trimmedText && !activeContexts.includes(trimmedText)) {
      setActiveContexts((prev) => [...prev, trimmedText]);
    }
  };

  const removeContext = (index: number) => {
    setActiveContexts((prev) => prev.filter((_, i) => i !== index));
  };

  const clearContexts = () => {
    setActiveContexts([]);
  };

  return (
    <ChatContext.Provider
      value={{
        isChatOpen,
        setIsChatOpen,
        activeContexts,
        addContext,
        removeContext,
        clearContexts,
      }}
    >
      {children}
    </ChatContext.Provider>
  );
}

export function useChatContext(): ChatContextType {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatContext must be used within a ChatProvider');
  }
  return context;
}
