import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';
import { ChatProvider } from '@site/src/context/ChatContext';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <ChatProvider>
      {children}
      <ChatWidget />
    </ChatProvider>
  );
}
