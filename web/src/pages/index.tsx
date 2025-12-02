import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

interface FeatureItem {
  title: string;
  emoji: string;
  description: string;
}

const features: FeatureItem[] = [
  {
    title: 'ROS 2 (Nervous System)',
    emoji: '🤖',
    description: 'Master Robot Operating System 2 fundamentals, nodes, topics, and real-time communication for robotics applications.',
  },
  {
    title: 'Simulation (Digital Twin)',
    emoji: '🌐',
    description: 'Learn Gazebo and simulation techniques to build and test robots in virtual environments before physical deployment.',
  },
  {
    title: 'Isaac Sim (Brain)',
    emoji: '🧠',
    description: 'Explore NVIDIA Isaac Sim for AI-powered robotics, physics simulation, and synthetic data generation.',
  },
  {
    title: 'VLA (Capstone)',
    emoji: '🎯',
    description: 'Build Vision-Language-Action models to create intelligent robots that understand and interact with the world.',
  },
];

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <h1 className={styles.heroTitle}>Physical AI & Humanoid Robotics</h1>
        <p className={styles.heroSubtitle}>From ROS 2 to VLA Models</p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/en/intro">
            Start Learning →
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({title, emoji, description}: FeatureItem) {
  return (
    <div className={styles.featureCard}>
      <div className={styles.featureEmoji}>{emoji}</div>
      <h3>{title}</h3>
      <p>{description}</p>
    </div>
  );
}

function FeatureGrid() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featureGrid}>
          {features.map((props, idx) => (
            <FeatureCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function TechStack() {
  return (
    <section className={styles.techStack}>
      <div className="container">
        <h2>Powered by Modern AI Stack</h2>
        <div className={styles.techStackGrid}>
          <div className={styles.techItem}>
            <span className={styles.techIcon}>✨</span>
            <span>Gemini 1.5 Flash</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techIcon}>🔍</span>
            <span>Qdrant Vector DB</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techIcon}>⚡</span>
            <span>FastAPI Backend</span>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Physical AI & Humanoid Robotics - Hardware-Adaptive Learning Platform">
      <HomepageHeader />
      <main>
        <FeatureGrid />
        <TechStack />
      </main>
    </Layout>
  );
}
