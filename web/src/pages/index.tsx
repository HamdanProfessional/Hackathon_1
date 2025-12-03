import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { useSession } from '../lib/auth-client';
import styles from './index.module.css';

interface FeatureItem {
  title: string;
  emoji: string;
  description: string;
}

interface LearningPathItem {
  step: number;
  title: string;
  description: string;
  duration: string;
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

const learningPath: LearningPathItem[] = [
  {
    step: 1,
    title: 'ROS 2 Foundations',
    description: 'Start with the basics of Robot Operating System 2, understanding nodes, topics, services, and actions.',
    duration: '4-6 weeks',
  },
  {
    step: 2,
    title: 'Robot Simulation',
    description: 'Learn to simulate robots in Gazebo, creating virtual environments and testing robot behaviors safely.',
    duration: '3-4 weeks',
  },
  {
    step: 3,
    title: 'AI-Powered Robotics',
    description: 'Dive into NVIDIA Isaac Sim for advanced physics simulation and synthetic data generation.',
    duration: '5-7 weeks',
  },
  {
    step: 4,
    title: 'Vision-Language-Action',
    description: 'Build intelligent robots that can see, understand language, and take actions in the real world.',
    duration: '6-8 weeks',
  },
];

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const { data: session } = useSession();

  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <h1 className={styles.heroTitle}>Physical AI & Humanoid Robotics</h1>
        <p className={styles.heroSubtitle}>From ROS 2 to Vision-Language-Action Models</p>
        <p className={styles.heroDescription}>
          Master robotics from fundamentals to advanced AI. Learn ROS 2, robot simulation,
          NVIDIA Isaac Sim, and build intelligent robots with VLA models.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/en/intro">
            Start Learning →
          </Link>
          {!session?.user && (
            <Link
              className="button button--secondary button--lg"
              to="/login">
              Sign Up Free
            </Link>
          )}
        </div>
        {session?.user && (
          <p className={styles.welcomeMessage}>
            Welcome back, <strong>{session.user.name}</strong>! Continue your journey.
          </p>
        )}
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

function PersonalizationHighlight() {
  return (
    <section className={styles.personalizationSection}>
      <div className="container">
        <div className={styles.personalizationContent}>
          <div className={styles.personalizationText}>
            <h2>Hardware-Adaptive Learning</h2>
            <p>
              Get personalized content tailored to your hardware setup. Whether you have
              a high-end GPU, NVIDIA Jetson, or just a laptop, we adapt code examples
              and explanations to match your resources.
            </p>
            <ul className={styles.benefitsList}>
              <li>GPU-optimized code for RTX 4090 users</li>
              <li>Edge-specific optimizations for Jetson devices</li>
              <li>CPU-friendly alternatives for laptop users</li>
              <li>Cloud-based solutions for Colab users</li>
            </ul>
          </div>
          <div className={styles.personalizationVisual}>
            <div className={styles.hardwareCard}>
              <span className={styles.hardwareIcon}>💻</span>
              <p>Laptop</p>
            </div>
            <div className={styles.hardwareCard}>
              <span className={styles.hardwareIcon}>🎮</span>
              <p>RTX 4090</p>
            </div>
            <div className={styles.hardwareCard}>
              <span className={styles.hardwareIcon}>🤖</span>
              <p>Jetson</p>
            </div>
            <div className={styles.hardwareCard}>
              <span className={styles.hardwareIcon}>☁️</span>
              <p>Cloud</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function LearningPath() {
  return (
    <section className={styles.learningPathSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Your Learning Journey</h2>
        <p className={styles.sectionSubtitle}>
          Follow our structured path from robotics fundamentals to advanced AI systems
        </p>
        <div className={styles.pathGrid}>
          {learningPath.map((item) => (
            <div key={item.step} className={styles.pathCard}>
              <div className={styles.pathNumber}>{item.step}</div>
              <h3>{item.title}</h3>
              <p>{item.description}</p>
              <div className={styles.pathDuration}>{item.duration}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function Statistics() {
  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className={styles.statsGrid}>
          <div className={styles.statCard}>
            <div className={styles.statNumber}>4</div>
            <div className={styles.statLabel}>Major Modules</div>
          </div>
          <div className={styles.statCard}>
            <div className={styles.statNumber}>50+</div>
            <div className={styles.statLabel}>Hands-on Projects</div>
          </div>
          <div className={styles.statCard}>
            <div className={styles.statNumber}>2</div>
            <div className={styles.statLabel}>Languages</div>
          </div>
          <div className={styles.statCard}>
            <div className={styles.statNumber}>AI</div>
            <div className={styles.statLabel}>Powered Chatbot</div>
          </div>
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
          <div className={styles.techItem}>
            <span className={styles.techIcon}>🔐</span>
            <span>Better-Auth</span>
          </div>
          <div className={styles.techItem}>
            <span className={styles.techIcon}>⚛️</span>
            <span>React + Docusaurus</span>
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
        <PersonalizationHighlight />
        <LearningPath />
        <Statistics />
        <TechStack />
      </main>
    </Layout>
  );
}
