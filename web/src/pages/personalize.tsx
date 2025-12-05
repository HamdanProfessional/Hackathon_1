import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { getCurrentUser, updateProfile, isAuthenticated } from '@site/src/lib/simple-auth-client';
import styles from './login.module.css';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function Personalize(): JSX.Element {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);

  // Profile fields
  const [hardwareBg, setHardwareBg] = useState<string>('');
  const [skillLevel, setSkillLevel] = useState<string>('');

  useEffect(() => {
    // Check if user is logged in
    if (ExecutionEnvironment.canUseDOM && !isAuthenticated()) {
      window.location.href = '/Hackathon_1/login';
      return;
    }

    // Load current profile
    const user = getCurrentUser();
    if (user) {
      setHardwareBg(user.hardware_bg || '');
      setSkillLevel(user.skill_level || '');
    }
  }, []);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setSuccess(false);
    setLoading(true);

    try {
      console.log('🔧 Updating profile:', { hardwareBg, skillLevel });

      const response = await updateProfile({
        hardware_bg: hardwareBg,
        skill_level: skillLevel,
      });

      console.log('📬 Profile update response:', response);

      if (!response.success) {
        setError(response.error || 'Profile update failed');
        setLoading(false);
        return;
      }

      setSuccess(true);
      setLoading(false);

      // Redirect to intro page after 2 seconds
      setTimeout(() => {
        if (ExecutionEnvironment.canUseDOM) {
          window.location.href = '/Hackathon_1/docs/en/intro';
        }
      }, 2000);
    } catch (err) {
      console.error('❌ Profile update error:', err);
      setError(err instanceof Error ? err.message : 'Profile update failed');
      setLoading(false);
    }
  };

  return (
    <Layout
      title="Personalize"
      description="Personalize your learning experience">
      <div className={styles.container}>
        <div className={styles.formCard}>
          <div className={styles.header}>
            <h1>Personalize Your Experience</h1>
            <p className={styles.subtitle}>
              Tell us about your hardware and experience level so we can tailor the content for you
            </p>
          </div>

          <form onSubmit={handleSubmit} className={styles.form}>
            {/* Hardware Background */}
            <div className={styles.formGroup}>
              <label className={styles.label}>
                What hardware do you have access to?
              </label>
              <div className={styles.radioGroup}>
                <label className={styles.radioOption}>
                  <input
                    type="radio"
                    name="hardware"
                    value="RTX4090"
                    checked={hardwareBg === 'RTX4090'}
                    onChange={(e) => setHardwareBg(e.target.value)}
                  />
                  <span>High-end GPU (RTX 4090, RTX 4080, etc.)</span>
                </label>
                <label className={styles.radioOption}>
                  <input
                    type="radio"
                    name="hardware"
                    value="Jetson"
                    checked={hardwareBg === 'Jetson'}
                    onChange={(e) => setHardwareBg(e.target.value)}
                  />
                  <span>NVIDIA Jetson (Orin, Xavier, Nano)</span>
                </label>
                <label className={styles.radioOption}>
                  <input
                    type="radio"
                    name="hardware"
                    value="Laptop"
                    checked={hardwareBg === 'Laptop'}
                    onChange={(e) => setHardwareBg(e.target.value)}
                  />
                  <span>Just my laptop/desktop CPU</span>
                </label>
                <label className={styles.radioOption}>
                  <input
                    type="radio"
                    name="hardware"
                    value="Cloud"
                    checked={hardwareBg === 'Cloud'}
                    onChange={(e) => setHardwareBg(e.target.value)}
                  />
                  <span>Cloud/Colab (no local hardware)</span>
                </label>
              </div>
            </div>

            {/* Skill Level */}
            <div className={styles.formGroup}>
              <label className={styles.label}>
                What's your experience level with robotics and AI?
              </label>
              <div className={styles.radioGroup}>
                <label className={styles.radioOption}>
                  <input
                    type="radio"
                    name="skill"
                    value="Beginner"
                    checked={skillLevel === 'Beginner'}
                    onChange={(e) => setSkillLevel(e.target.value)}
                  />
                  <span>Beginner - New to robotics and AI</span>
                </label>
                <label className={styles.radioOption}>
                  <input
                    type="radio"
                    name="skill"
                    value="Advanced"
                    checked={skillLevel === 'Advanced'}
                    onChange={(e) => setSkillLevel(e.target.value)}
                  />
                  <span>Advanced - Have some experience with robotics/AI</span>
                </label>
              </div>
            </div>

            <div className={styles.hint}>
              <p>
                <strong>Why personalize?</strong> We'll show you code examples and explanations
                optimized for your hardware and skill level. You can update this anytime.
              </p>
            </div>

            {error && (
              <div className={styles.error}>
                <strong>⚠️ Error:</strong> {error}
              </div>
            )}

            {success && (
              <div style={{ padding: '12px', background: '#d4edda', color: '#155724', borderRadius: '6px', marginBottom: '16px' }}>
                <strong>✅ Success!</strong> Profile updated. Redirecting...
              </div>
            )}

            <button
              type="submit"
              disabled={loading || !hardwareBg || !skillLevel}
              className={styles.submitButton}
            >
              {loading ? 'Saving...' : 'Save Preferences'}
            </button>
          </form>
        </div>

        <div className={styles.features}>
          <h2>🤖 AI Content Generation</h2>
          <p style={{ marginBottom: '20px', lineHeight: '1.6' }}>
            Once you save your preferences, you can generate personalized tutorials, code examples,
            and explanations tailored to your <strong>{hardwareBg || 'hardware'}</strong> and <strong>{skillLevel || 'skill level'}</strong>!
          </p>

          {hardwareBg && skillLevel ? (
            <div style={{
              background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
              color: 'white',
              padding: '20px',
              borderRadius: '12px',
              marginBottom: '20px'
            }}>
              <h3 style={{ marginTop: 0, marginBottom: '12px' }}>✨ Your Personalized Context</h3>
              <div style={{ background: 'rgba(255,255,255,0.1)', padding: '15px', borderRadius: '8px' }}>
                <p style={{ margin: '0 0 8px 0' }}><strong>Hardware:</strong> {hardwareBg}</p>
                <p style={{ margin: 0 }}><strong>Level:</strong> {skillLevel}</p>
              </div>
              <p style={{ marginTop: '15px', marginBottom: 0, fontSize: '14px', opacity: 0.9 }}>
                💡 <em>AI will generate content optimized for your setup! Save your preferences first, then start exploring the textbook.</em>
              </p>
            </div>
          ) : (
            <div style={{
              background: '#f8f9fa',
              border: '2px dashed #dee2e6',
              padding: '20px',
              borderRadius: '12px',
              textAlign: 'center',
              color: '#6c757d'
            }}>
              <p style={{ margin: 0 }}>
                <strong>👆 Select your hardware and skill level above</strong><br />
                to unlock AI-powered personalized content generation!
              </p>
            </div>
          )}

          <h3 style={{ marginTop: '30px' }}>What You Can Generate:</h3>
          <ul>
            <li>
              <span className={styles.icon}>📝</span>
              <strong>Custom Tutorials:</strong> Step-by-step guides for your hardware
            </li>
            <li>
              <span className={styles.icon}>💻</span>
              <strong>Code Examples:</strong> Optimized for your specific setup
            </li>
            <li>
              <span className={styles.icon}>📖</span>
              <strong>Explanations:</strong> Matched to your skill level
            </li>
            <li>
              <span className={styles.icon}>⚡</span>
              <strong>Performance Tips:</strong> Hardware-specific optimizations
            </li>
          </ul>
        </div>
      </div>
    </Layout>
  );
}
