/**
 * Session Debug Component
 *
 * Shows detailed session information for debugging
 * Only visible in development or when ?debug=true is in URL
 */

import React, { useEffect, useState } from 'react';
import { useSession } from '@site/src/lib/auth-client';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function SessionDebug() {
  const { data: session, isPending, error } = useSession();
  const [cookies, setCookies] = useState<string>('');
  const [showDebug, setShowDebug] = useState(false);

  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      // Check if debug mode is enabled via URL param
      const params = new URLSearchParams(window.location.search);
      setShowDebug(params.get('debug') === 'true' || process.env.NODE_ENV === 'development');

      // Get all cookies
      setCookies(document.cookie);
    }
  }, []);

  if (!showDebug) {
    return null;
  }

  return (
    <div
      style={{
        position: 'fixed',
        bottom: '80px',
        right: '20px',
        background: 'rgba(0, 0, 0, 0.9)',
        color: '#fff',
        padding: '15px',
        borderRadius: '8px',
        maxWidth: '400px',
        fontSize: '12px',
        fontFamily: 'monospace',
        zIndex: 9998,
        maxHeight: '300px',
        overflow: 'auto',
      }}
    >
      <h4 style={{ margin: '0 0 10px 0', color: '#4CAF50' }}>Session Debug Info</h4>

      <div style={{ marginBottom: '10px' }}>
        <strong>Session Status:</strong>
        <pre style={{ margin: '5px 0', whiteSpace: 'pre-wrap' }}>
          {isPending ? '⏳ Loading...' : session ? '✅ Authenticated' : '❌ Not authenticated'}
        </pre>
      </div>

      {error && (
        <div style={{ marginBottom: '10px' }}>
          <strong>Error:</strong>
          <pre style={{ margin: '5px 0', color: '#f44336', whiteSpace: 'pre-wrap' }}>
            {JSON.stringify(error, null, 2)}
          </pre>
        </div>
      )}

      {session && (
        <div style={{ marginBottom: '10px' }}>
          <strong>User Data:</strong>
          <pre style={{ margin: '5px 0', whiteSpace: 'pre-wrap' }}>
            {JSON.stringify(session.user, null, 2)}
          </pre>
        </div>
      )}

      <div style={{ marginBottom: '10px' }}>
        <strong>Cookies:</strong>
        <pre style={{ margin: '5px 0', whiteSpace: 'pre-wrap', wordBreak: 'break-all' }}>
          {cookies || 'No cookies found'}
        </pre>
      </div>

      <div style={{ marginBottom: '10px' }}>
        <strong>Auth Base URL:</strong>
        <pre style={{ margin: '5px 0', whiteSpace: 'pre-wrap' }}>
          https://auth-qmb4qcc6u-hamdanprofessionals-projects.vercel.app
        </pre>
      </div>

      <button
        onClick={() => setShowDebug(false)}
        style={{
          background: '#f44336',
          color: '#fff',
          border: 'none',
          padding: '5px 10px',
          borderRadius: '4px',
          cursor: 'pointer',
          fontSize: '12px',
        }}
      >
        Close Debug
      </button>
    </div>
  );
}
