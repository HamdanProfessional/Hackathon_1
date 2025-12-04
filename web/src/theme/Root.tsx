import React, { useEffect, useState } from 'react';
import ChatWidget from '@site/src/components/ChatWidget';
import SessionDebug from '@site/src/components/SessionDebug';
import { ChatProvider } from '@site/src/context/ChatContext';

// Default implementation, that you can customize
export default function Root({children}) {
  // Only render client-side components to avoid hydration errors
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  return (
    <ChatProvider>
      {children}
      {isClient && (
        <>
          <ChatWidget />
          <SessionDebug />
        </>
      )}
    </ChatProvider>
  );
}
