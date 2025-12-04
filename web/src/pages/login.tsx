import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { authClient } from '@site/src/lib/auth-client';
import styles from './login.module.css';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// Valid values from database schema
type HardwareOption = 'RTX4090' | 'Jetson' | 'Laptop' | 'Cloud';
type SkillLevel = 'Beginner' | 'Advanced';

// Questions to understand user's background
const HARDWARE_QUESTIONS = [
  {
    question: 'What hardware do you have access to for robotics projects?',
    options: [
      { value: 'RTX4090', label: 'High-end GPU (RTX 4090, RTX 4080, etc.)' },
      { value: 'Jetson', label: 'NVIDIA Jetson (Orin, Xavier, Nano)' },
      { value: 'Laptop', label: 'Just my laptop/desktop CPU' },
      { value: 'Cloud', label: 'Cloud/Colab (no local hardware)' },
    ]
  }
];

const EXPERIENCE_QUESTIONS = [
  {
    question: 'Have you worked with robotics or ROS before?',
    options: [
      { value: 'none', label: 'No, I\'m completely new to this' },
      { value: 'basic', label: 'I\'ve done some tutorials or simple projects' },
      { value: 'intermediate', label: 'I\'ve built a few robots or worked with ROS' },
      { value: 'advanced', label: 'I have professional experience' },
    ]
  },
  {
    question: 'How comfortable are you with Python programming?',
    options: [
      { value: 'beginner', label: 'Just starting out or basic knowledge' },
      { value: 'intermediate', label: 'Can write scripts and understand OOP' },
      { value: 'advanced', label: 'Proficient with advanced concepts' },
    ]
  }
];

export default function Login(): JSX.Element {
  const [isSignUp, setIsSignUp] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Form state
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');

  // Conversational question answers - default to first option
  const [hardwareAnswer, setHardwareAnswer] = useState<string>('Laptop');
  const [roboticsExperience, setRoboticsExperience] = useState<string>('none');
  const [pythonSkill, setPythonSkill] = useState<string>('beginner');

  // Derive hardware_bg from hardware answer (direct mapping)
  const deriveHardwareBg = (): HardwareOption => {
    // Always return a valid value, defaulting to Laptop
    return (hardwareAnswer as HardwareOption) || 'Laptop';
  };

  // Derive skill_level from robotics experience and Python skill
  const deriveSkillLevel = (): SkillLevel => {
    // Beginner: New to robotics (none/basic) AND beginner at Python
    // Advanced: Otherwise
    if ((roboticsExperience === 'none' || roboticsExperience === 'basic') &&
        pythonSkill === 'beginner') {
      return 'Beginner';
    }
    return 'Advanced';
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      if (isSignUp) {
        // Derive values from conversational answers
        const hardware_bg = deriveHardwareBg();
        const skill_level = deriveSkillLevel();

        console.log('🔐 Sign up attempt with:', {
          email,
          name,
          hardware_bg,
          skill_level,
          hasPassword: !!password,
        });

        // Sign up with hardware preferences and skill level
        const response = await authClient.signUp.email({
          email,
          password,
          name,
          // @ts-ignore - Better-Auth allows custom fields
          hardware_bg,
          skill_level,
        });

        console.log('📬 Sign up response:', response);

        // Check for errors in response
        if (response.error) {
          console.error('❌ Sign up error:', response.error);
          setError(response.error.message || 'Sign up failed');
          setLoading(false);
          return;
        }

        // Redirect to first chapter only on success
        if (ExecutionEnvironment.canUseDOM) {
          window.location.href = '/Hackathon_1/docs/en/intro';
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
          window.location.href = '/Hackathon_1/docs/en/intro';
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
                {/* Hardware Question */}
                <div className={styles.formGroup}>
                  <label className={styles.label}>
                    {HARDWARE_QUESTIONS[0].question}
                  </label>
                  <div className={styles.radioGroup}>
                    {HARDWARE_QUESTIONS[0].options.map((option) => (
                      <label key={option.value} className={styles.radioOption}>
                        <input
                          type="radio"
                          name="hardware"
                          value={option.value}
                          checked={hardwareAnswer === option.value}
                          onChange={(e) => setHardwareAnswer(e.target.value)}
                        />
                        <span>{option.label}</span>
                      </label>
                    ))}
                  </div>
                </div>

                {/* Robotics Experience Question */}
                <div className={styles.formGroup}>
                  <label className={styles.label}>
                    {EXPERIENCE_QUESTIONS[0].question}
                  </label>
                  <div className={styles.radioGroup}>
                    {EXPERIENCE_QUESTIONS[0].options.map((option) => (
                      <label key={option.value} className={styles.radioOption}>
                        <input
                          type="radio"
                          name="roboticsExperience"
                          value={option.value}
                          checked={roboticsExperience === option.value}
                          onChange={(e) => setRoboticsExperience(e.target.value)}
                        />
                        <span>{option.label}</span>
                      </label>
                    ))}
                  </div>
                </div>

                {/* Python Skill Question */}
                <div className={styles.formGroup}>
                  <label className={styles.label}>
                    {EXPERIENCE_QUESTIONS[1].question}
                  </label>
                  <div className={styles.radioGroup}>
                    {EXPERIENCE_QUESTIONS[1].options.map((option) => (
                      <label key={option.value} className={styles.radioOption}>
                        <input
                          type="radio"
                          name="pythonSkill"
                          value={option.value}
                          checked={pythonSkill === option.value}
                          onChange={(e) => setPythonSkill(e.target.value)}
                        />
                        <span>{option.label}</span>
                      </label>
                    ))}
                  </div>
                </div>

                <div className={styles.hint}>
                  <p style={{ marginBottom: '8px' }}>
                    <strong>Why we ask:</strong> We'll personalize code examples and explanations based on your answers.
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
