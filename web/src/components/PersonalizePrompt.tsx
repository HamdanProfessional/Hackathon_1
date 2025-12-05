/**
 * PersonalizePrompt Component
 *
 * Shows a personalization prompt in textbook content
 * Encourages users to personalize their learning experience
 */

import React, { useState, useEffect } from 'react';
import { getCurrentUser, isAuthenticated } from '@site/src/lib/simple-auth-client';
import Link from '@docusaurus/Link';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function PersonalizePrompt() {
  const [user, setUser] = useState(null);
  const [isLoggedIn, setIsLoggedIn] = useState(false);

  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;

    setIsLoggedIn(isAuthenticated());
    setUser(getCurrentUser());
  }, []);

  // Don't show if not logged in
  if (!ExecutionEnvironment.canUseDOM || !isLoggedIn) {
    return null;
  }

  // Check if user has personalized
  const hasPersonalized = user?.hardware_bg && user?.skill_level;

  if (hasPersonalized) {
    // Show current personalization status
    return (
      <div style={{
        background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
        color: 'white',
        padding: '20px',
        borderRadius: '12px',
        marginBottom: '24px',
        boxShadow: '0 4px 6px rgba(0,0,0,0.1)'
      }}>
        <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', flexWrap: 'wrap', gap: '16px' }}>
          <div>
            <h3 style={{ margin: '0 0 8px 0', fontSize: '18px' }}>
              ✨ Personalized for You
            </h3>
            <p style={{ margin: 0, opacity: 0.9 }}>
              <strong>Hardware:</strong> {user.hardware_bg} | <strong>Level:</strong> {user.skill_level}
            </p>
          </div>
          <Link
            to="/Hackathon_1/personalize"
            className="button button--outline"
            style={{
              color: 'white',
              borderColor: 'white',
              '--ifm-button-color': 'white'
            }}
          >
            Update Preferences
          </Link>
        </div>
      </div>
    );
  }

  // Show personalization prompt
  return (
    <div style={{
      background: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)',
      color: 'white',
      padding: '24px',
      borderRadius: '12px',
      marginBottom: '24px',
      boxShadow: '0 4px 6px rgba(0,0,0,0.1)'
    }}>
      <h3 style={{ margin: '0 0 12px 0', fontSize: '20px' }}>
        🎯 Get Personalized Content
      </h3>
      <p style={{ margin: '0 0 16px 0', opacity: 0.95, lineHeight: '1.6' }}>
        Tell us about your hardware and experience level to get code examples and explanations
        tailored specifically for you. You can even generate AI-powered content based on your setup!
      </p>
      <Link
        to="/Hackathon_1/personalize"
        className="button button--lg"
        style={{
          background: 'white',
          color: '#f5576c',
          fontWeight: 'bold'
        }}
      >
        Personalize My Experience →
      </Link>
    </div>
  );
}
