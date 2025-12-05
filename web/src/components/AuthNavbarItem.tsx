/**
 * AuthNavbarItem Component
 *
 * Dynamic authentication button for Docusaurus navbar
 * Shows login button for guests, user info + logout for authenticated users
 */

import React, { useState, useEffect } from 'react';
import { getCurrentUser, logout, isAuthenticated } from '@site/src/lib/simple-auth-client';
import Link from '@docusaurus/Link';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function AuthNavbarItem() {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  // Check authentication state on mount and when storage changes
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) {
      setLoading(false);
      return;
    }

    const checkAuth = () => {
      const currentUser = getCurrentUser();
      setUser(currentUser);
      setLoading(false);
      console.log('🔐 Auth state:', { user: currentUser, isAuthenticated: isAuthenticated() });
    };

    // Check immediately
    checkAuth();

    // Listen for storage changes (in case user logs in/out in another tab)
    const handleStorageChange = (e: StorageEvent) => {
      if (e.key === 'auth_token' || e.key === 'auth_user') {
        checkAuth();
      }
    };

    window.addEventListener('storage', handleStorageChange);

    return () => {
      window.removeEventListener('storage', handleStorageChange);
    };
  }, []);

  const handleLogout = () => {
    console.log('🔄 Logging out...');
    logout();
    setUser(null);

    // Redirect to home
    if (ExecutionEnvironment.canUseDOM) {
      window.location.href = '/Hackathon_1/';
    }
  };

  if (loading) {
    return (
      <div className="navbar__item">
        <span style={{ opacity: 0.6 }}>Loading...</span>
      </div>
    );
  }

  if (user) {
    return (
      <div className="navbar__item" style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
        <span style={{ fontSize: '14px' }}>
          Hi, <strong>{user.email}</strong>
        </span>
        <button
          className="button button--secondary button--sm"
          onClick={handleLogout}
          style={{ cursor: 'pointer' }}
        >
          Logout
        </button>
      </div>
    );
  }

  return (
    <Link
      to="/Hackathon_1/login"
      className="button button--primary button--sm navbar__item"
    >
      Login
    </Link>
  );
}
