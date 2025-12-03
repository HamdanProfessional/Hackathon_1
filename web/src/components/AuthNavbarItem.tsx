/**
 * AuthNavbarItem Component
 *
 * Dynamic authentication button for Docusaurus navbar
 * Shows login button for guests, user info + logout for authenticated users
 */

import React from 'react';
import { useSession, authClient } from '@site/src/lib/auth-client';
import Link from '@docusaurus/Link';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function AuthNavbarItem() {
  const { data: session, isPending, error } = useSession();

  // Debug logging for session state
  React.useEffect(() => {
    console.log('🔐 AuthNavbarItem Debug:', {
      session: session,
      isPending: isPending,
      error: error,
      user: session?.user,
    });
  }, [session, isPending, error]);

  if (isPending) {
    return (
      <div className="navbar__item">
        <span style={{ opacity: 0.6 }}>Loading...</span>
      </div>
    );
  }

  if (session?.user) {
    return (
      <div className="navbar__item" style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
        <span style={{ fontSize: '14px' }}>
          Hi, <strong>{session.user.name || session.user.email}</strong>
        </span>
        <button
          className="button button--secondary button--sm"
          onClick={async () => {
            try {
              console.log('🔄 Attempting to sign out...');
              const result = await authClient.signOut({
                fetchOptions: {
                  onSuccess: () => {
                    console.log('✅ Sign out success callback triggered');
                  },
                  onError: (ctx) => {
                    console.error('❌ Sign out error callback:', ctx.error);
                  }
                }
              });
              console.log('✅ Sign out result:', result);

              // Force reload to clear all state
              if (ExecutionEnvironment.canUseDOM) {
                // Clear any cached session data
                sessionStorage.clear();
                // Redirect to home
                window.location.href = '/';
              }
            } catch (error) {
              console.error('❌ Logout error:', error);
              // Still redirect even if there's an error
              if (ExecutionEnvironment.canUseDOM) {
                sessionStorage.clear();
                window.location.href = '/';
              }
            }
          }}
          style={{ cursor: 'pointer' }}
        >
          Logout
        </button>
      </div>
    );
  }

  return (
    <Link
      to="/login"
      className="button button--primary button--sm navbar__item"
    >
      Login
    </Link>
  );
}
