import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { signUp, login } from '@site/src/lib/simple-auth-client';
import styles from './login.module.css';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function Login(): JSX.Element {
  const [isSignUp, setIsSignUp] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Form state - simplified to just email and password
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      if (isSignUp) {
        console.log('🔐 Sign up attempt:', email);

        // Call our custom signup endpoint
        const response = await signUp(email, password);

        console.log('📬 Sign up response:', response);

        if (!response.success) {
          setError(response.error || 'Sign up failed');
          setLoading(false);
          return;
        }

        // Redirect to first chapter on success
        if (ExecutionEnvironment.canUseDOM) {
          window.location.href = '/Hackathon_1/docs/en/intro';
        }
      } else {
        console.log('🔐 Login attempt:', email);

        // Call our custom login endpoint
        const response = await login(email, password);

        console.log('📬 Login response:', response);

        if (!response.success) {
          setError(response.error || 'Login failed. Please check your credentials.');
          setLoading(false);
          return;
        }

        // Redirect to first chapter on success
        if (ExecutionEnvironment.canUseDOM) {
          window.location.href = '/Hackathon_1/docs/en/intro';
        }
      }
    } catch (err) {
      console.error('❌ Authentication error:', err);
      setError(err instanceof Error ? err.message : 'Authentication failed');
      setLoading(false);
    }
  };

  return (
    <Layout
      title="Login"
      description="Login or sign up to access hardware-personalized content">
      <div className={styles.container}>
        <div className={styles.formCard}>
          <div className={styles.header}>
            <h1>{isSignUp ? 'Create Account' : 'Welcome Back'}</h1>
            <p className={styles.subtitle}>
              {isSignUp
                ? 'Sign up to get personalized robotics content'
                : 'Sign in to access your personalized content'}
            </p>
          </div>

          <form onSubmit={handleSubmit} className={styles.form}>
            <div className={styles.formGroup}>
              <label htmlFor="email" className={styles.label}>
                Email
              </label>
              <input
                id="email"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="you@example.com"
                required
                className={styles.input}
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="password" className={styles.label}>
                Password
              </label>
              <input
                id="password"
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="••••••••"
                required
                minLength={8}
                className={styles.input}
              />
            </div>


            {error && (
              <div className={styles.error}>
                <strong>⚠️ Error:</strong> {error}
              </div>
            )}

            <button
              type="submit"
              disabled={loading}
              className={styles.submitButton}
            >
              {loading ? 'Loading...' : isSignUp ? 'Create Account' : 'Sign In'}
            </button>
          </form>

          <div className={styles.footer}>
            <button
              onClick={() => {
                setIsSignUp(!isSignUp);
                setError(null);
              }}
              className={styles.toggleButton}
            >
              {isSignUp
                ? 'Already have an account? Sign in'
                : "Don't have an account? Sign up"}
            </button>
          </div>
        </div>

        <div className={styles.features}>
          <h2>Why Sign Up?</h2>
          <ul>
            <li>
              <span className={styles.icon}>🎯</span>
              <strong>Personalized Content:</strong> Code examples adapted to your hardware
            </li>
            <li>
              <span className={styles.icon}>💾</span>
              <strong>Save Progress:</strong> Track your learning journey
            </li>
            <li>
              <span className={styles.icon}>🤖</span>
              <strong>AI Assistant:</strong> Get help from our RAG-powered chatbot
            </li>
            <li>
              <span className={styles.icon}>📚</span>
              <strong>Bilingual Support:</strong> Access content in English and Urdu
            </li>
          </ul>
        </div>
      </div>
    </Layout>
  );
}
