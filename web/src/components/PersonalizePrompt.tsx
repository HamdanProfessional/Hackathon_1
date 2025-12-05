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
  const [isGenerating, setIsGenerating] = useState(false);
  const [generatedContent, setGeneratedContent] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;

    setIsLoggedIn(isAuthenticated());
    setUser(getCurrentUser());
  }, []);

  const handlePersonalize = async () => {
    setIsGenerating(true);
    setError(null);
    setGeneratedContent(null);

    try {
      // Get current page content
      const pageContent = document.querySelector('article')?.innerText || '';
      const pageTitle = document.querySelector('h1')?.innerText || 'Introduction';

      console.log('🤖 Generating personalized content for:', { pageTitle, hardware: user?.hardware_bg, skill: user?.skill_level });

      const response = await fetch('https://auth-hamdanprofessionals-projects.vercel.app/api/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          pageContent,
          pageTitle,
          hardware_bg: user?.hardware_bg,
          skill_level: user?.skill_level,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to generate personalized content');
      }

      console.log('✅ Generated content:', data);
      setGeneratedContent(data.content);
    } catch (err) {
      console.error('❌ Personalization error:', err);
      setError(err instanceof Error ? err.message : 'Failed to generate content');
    } finally {
      setIsGenerating(false);
    }
  };

  // Don't show if not logged in
  if (!ExecutionEnvironment.canUseDOM || !isLoggedIn) {
    return null;
  }

  // Check if user has personalized
  const hasPersonalized = user?.hardware_bg && user?.skill_level;

  if (hasPersonalized) {
    // Show current personalization status
    return (
      <>
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
            <div style={{ display: 'flex', gap: '10px', flexWrap: 'wrap' }}>
              <button
                onClick={handlePersonalize}
                disabled={isGenerating}
                className="button button--primary"
                style={{
                  background: 'white',
                  color: '#667eea',
                  fontWeight: 'bold',
                  border: 'none',
                  cursor: isGenerating ? 'not-allowed' : 'pointer',
                  opacity: isGenerating ? 0.7 : 1
                }}
              >
                {isGenerating ? '🤖 Generating...' : '🚀 Start Personalization'}
              </button>
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
        </div>

        {/* Generated Content Display */}
        {generatedContent && (
          <div style={{
            background: '#f8f9fa',
            border: '2px solid #667eea',
            padding: '24px',
            borderRadius: '12px',
            marginBottom: '24px',
            maxHeight: '600px',
            overflowY: 'auto'
          }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '16px' }}>
              <h3 style={{ margin: 0, color: '#667eea' }}>
                🤖 AI-Generated Personalized Content
              </h3>
              <button
                onClick={() => setGeneratedContent(null)}
                style={{
                  background: 'none',
                  border: 'none',
                  fontSize: '24px',
                  cursor: 'pointer',
                  color: '#666'
                }}
                title="Close"
              >
                ×
              </button>
            </div>
            <div
              style={{
                lineHeight: '1.8',
                color: '#333',
                whiteSpace: 'pre-wrap'
              }}
              dangerouslySetInnerHTML={{ __html: generatedContent }}
            />
          </div>
        )}

        {/* Error Display */}
        {error && (
          <div style={{
            background: '#f8d7da',
            border: '1px solid #f5c6cb',
            color: '#721c24',
            padding: '16px',
            borderRadius: '8px',
            marginBottom: '24px'
          }}>
            <strong>⚠️ Error:</strong> {error}
          </div>
        )}
      </>
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
