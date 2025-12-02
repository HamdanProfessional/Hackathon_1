import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { authClient } from '@site/src/lib/auth-client';
import styles from './login.module.css';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

type HardwareOption = 'RTX 4090' | 'Jetson Orin' | 'Laptop CPU' | 'Google Colab';
type SkillLevel = 'Beginner' | 'Advanced';

const HARDWARE_OPTIONS: HardwareOption[] = [
  'RTX 4090',
  'Jetson Orin',
  'Laptop CPU',
  'Google Colab'
];

const SKILL_LEVEL_OPTIONS: SkillLevel[] = ['Beginner', 'Advanced'];

export default function Login(): JSX.Element {
  const [isSignUp, setIsSignUp] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Form state
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [hardwareBg, setHardwareBg] = useState<HardwareOption>('Laptop CPU');
  const [skillLevel, setSkillLevel] = useState<SkillLevel>('Beginner');

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      if (isSignUp) {
        // Sign up with hardware preferences and skill level
        const response = await authClient.signUp.email({
          email,
          password,
          name,
          // @ts-ignore - Better-Auth allows custom fields
          hardwareBg,
          skillLevel,
        });

        // Check for errors in response
        if (response.error) {
          setError(response.error.message || 'Sign up failed');
          setLoading(false);
          return;
        }

        // Redirect to first chapter only on success
        if (ExecutionEnvironment.canUseDOM) {
          window.location.href = '/docs/en/intro';
        }
      } else {
        // Sign in
        const response = await authClient.signIn.email({
          email,
          password,
        });

        // Check for errors in response
        if (response.error) {
          setError(response.error.message || 'Login failed. Please check your credentials.');
          setLoading(false);
          return;
        }

        // Redirect to first chapter only on success
        if (ExecutionEnvironment.canUseDOM) {
          window.location.href = '/docs/en/intro';
        }
      }
    } catch (err) {
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
            {isSignUp && (
              <div className={styles.formGroup}>
                <label htmlFor="name" className={styles.label}>
                  Name
                </label>
                <input
                  id="name"
                  type="text"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  placeholder="Your name"
                  required={isSignUp}
                  className={styles.input}
                />
              </div>
            )}

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

            {isSignUp && (
              <>
                <div className={styles.formGroup}>
                  <label htmlFor="hardware" className={styles.label}>
                    Hardware Setup
                    <span className={styles.labelHint}>
                      (We'll personalize content for your hardware)
                    </span>
                  </label>
                  <select
                    id="hardware"
                    value={hardwareBg}
                    onChange={(e) => setHardwareBg(e.target.value as HardwareOption)}
                    required={isSignUp}
                    className={styles.select}
                  >
                    {HARDWARE_OPTIONS.map((option) => (
                      <option key={option} value={option}>
                        {option}
                      </option>
                    ))}
                  </select>
                  <p className={styles.hint}>
                    Choose the hardware you'll be using for robotics projects. This helps us
                    tailor code examples and performance guidance.
                  </p>
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="skillLevel" className={styles.label}>
                    Skill Level
                    <span className={styles.labelHint}>
                      (Adjusts content complexity)
                    </span>
                  </label>
                  <select
                    id="skillLevel"
                    value={skillLevel}
                    onChange={(e) => setSkillLevel(e.target.value as SkillLevel)}
                    required={isSignUp}
                    className={styles.select}
                  >
                    {SKILL_LEVEL_OPTIONS.map((option) => (
                      <option key={option} value={option}>
                        {option}
                      </option>
                    ))}
                  </select>
                  <p className={styles.hint}>
                    <strong>Beginner:</strong> Simple explanations with analogies, focus on "why"
                    <br />
                    <strong>Advanced:</strong> Technical depth, performance optimization, implementation details
                  </p>
                </div>
              </>
            )}

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
