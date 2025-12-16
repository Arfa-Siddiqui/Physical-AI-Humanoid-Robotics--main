import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroLeft}>
            <div className={styles.badge}>
              🤖 Advanced Robotics Education
            </div>
            <h1 className={styles.heroTitle}>
              Physical AI &<br/>
              Humanoid Robotics
            </h1>
            <p className={styles.heroSubtitle}>
              From Digital Intelligence to Embodied Action
            </p>
            <p className={styles.heroDescription}>
              Master the complete stack: ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action systems.
              Build autonomous humanoid robots that perceive, reason, and act in the physical world.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro/what-is-physical-ai">
                Start Learning →
              </Link>
              <Link
                className="button button--outline button--secondary button--lg"
                to="https://github.com/yourusername/Physical-AI-Humanoid-Robotics">
                GitHub
              </Link>
            </div>
          </div>
          <div className={styles.heroRight}>
            <div className={styles.heroImageContainer}>
              <svg viewBox="0 0 400 400" className={styles.heroImage}>
                <defs>
                  <linearGradient id="robotGrad" x1="0%" y1="0%" x2="100%" y2="100%">
                    <stop offset="0%" style={{stopColor: '#3b82f6', stopOpacity: 1}} />
                    <stop offset="100%" style={{stopColor: '#8b5cf6', stopOpacity: 1}} />
                  </linearGradient>
                  <filter id="glow">
                    <feGaussianBlur stdDeviation="4" result="coloredBlur"/>
                    <feMerge>
                      <feMergeNode in="coloredBlur"/>
                      <feMergeNode in="SourceGraphic"/>
                    </feMerge>
                  </filter>
                </defs>

                {/* Robot Body */}
                <g filter="url(#glow)">
                  <rect x="150" y="80" width="100" height="120" rx="15" fill="url(#robotGrad)" opacity="0.9"/>
                  <circle cx="175" cy="120" r="8" fill="#22d3ee"/>
                  <circle cx="225" cy="120" r="8" fill="#22d3ee"/>

                  {/* Head */}
                  <rect x="165" y="40" width="70" height="60" rx="10" fill="url(#robotGrad)" opacity="0.95"/>
                  <circle cx="185" cy="60" r="6" fill="#60a5fa"/>
                  <circle cx="215" cy="60" r="6" fill="#60a5fa"/>

                  {/* Arms */}
                  <rect x="100" y="100" width="40" height="80" rx="12" fill="url(#robotGrad)" opacity="0.8"/>
                  <rect x="260" y="100" width="40" height="80" rx="12" fill="url(#robotGrad)" opacity="0.8"/>

                  {/* Legs */}
                  <rect x="160" y="210" width="35" height="100" rx="10" fill="url(#robotGrad)" opacity="0.85"/>
                  <rect x="205" y="210" width="35" height="100" rx="10" fill="url(#robotGrad)" opacity="0.85"/>
                </g>

                {/* Circuit elements */}
                <g opacity="0.6" stroke="#60a5fa" strokeWidth="2" fill="none">
                  <circle cx="80" cy="80" r="20"/>
                  <circle cx="320" cy="280" r="25"/>
                  <path d="M 50 200 L 80 200 L 80 240"/>
                  <path d="M 320 150 L 350 150 L 350 200"/>
                  <circle cx="50" cy="200" r="5" fill="#22d3ee"/>
                  <circle cx="320" cy="150" r="5" fill="#a78bfa"/>
                </g>
              </svg>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({icon, title, description}) {
  return (
    <div className={styles.featureCard}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3>{title}</h3>
      <p>{description}</p>
    </div>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Master Physical AI and Humanoid Robotics with ROS 2, NVIDIA Isaac, and VLA systems">
      <HomepageHeader />

      <main className={styles.mainContent}>
        {/* What You'll Master */}
        <section className={styles.featuresSection}>
          <div className="container">
            <h2 className={styles.sectionTitle}>What You'll Master</h2>
            <div className={styles.featuresGrid}>
              <FeatureCard
                icon="🤖"
                title="ROS 2 Fundamentals"
                description="Master nodes, topics, services, actions, and URDF modeling. Build the robotic nervous system."
              />
              <FeatureCard
                icon="🌐"
                title="Digital Twins"
                description="Create physics-accurate simulations in Gazebo and Unity. Test before deploying to hardware."
              />
              <FeatureCard
                icon="🧠"
                title="NVIDIA Isaac"
                description="GPU-accelerated perception with Isaac ROS. Real-time SLAM, object detection, and Nav2 planning."
              />
              <FeatureCard
                icon="🎯"
                title="Vision-Language-Action"
                description="Integrate Whisper and GPT-4. Build voice-commanded autonomous systems that understand language."
              />
            </div>
          </div>
        </section>

        {/* Tech Stack */}
        <section className={styles.techSection}>
          <div className="container">
            <h2 className={styles.sectionTitle}>The Complete Technology Stack</h2>
            <div className={styles.techStack}>
              <div className={styles.techLayer}>
                <div className={styles.techBadge}>Module 4</div>
                <h4>Vision-Language-Action (Whisper + LLM)</h4>
              </div>
              <div className={styles.techLayer}>
                <div className={styles.techBadge}>Module 3</div>
                <h4>Isaac ROS (Perception) + Nav2 (Planning)</h4>
              </div>
              <div className={styles.techLayer}>
                <div className={styles.techBadge}>Module 2</div>
                <h4>Digital Twins (Gazebo, Unity, Isaac Sim)</h4>
              </div>
              <div className={styles.techLayer}>
                <div className={styles.techBadge}>Module 1</div>
                <h4>ROS 2 Middleware (Topics, Services, Actions)</h4>
              </div>
              <div className={styles.techLayer}>
                <div className={styles.techBadge}>Foundations</div>
                <h4>Sensors, Actuators, Hardware</h4>
              </div>
            </div>
          </div>
        </section>

        {/* Bonus Features */}
        <section className={styles.bonusSection}>
          <div className="container">
            <div className={styles.bonusBox}>
              <h2>📚 Comprehensive Learning Resources</h2>
              <div className={styles.bonusGrid}>
                <div className={styles.bonusCard}>
                  <h4>26 Detailed Chapters</h4>
                  <p>Progressive learning from basics to advanced topics</p>
                </div>
                <div className={styles.bonusCard}>
                  <h4>Hands-On Code Examples</h4>
                  <p>Copy-paste ready Python, C++, and YAML code</p>
                </div>
                <div className={styles.bonusCard}>
                  <h4>Complete Capstone Project</h4>
                  <p>Build a voice-commanded fetch-and-deliver robot</p>
                </div>
                <div className={styles.bonusCard}>
                  <h4>Production-Ready Patterns</h4>
                  <p>Best practices for real-world deployment</p>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* CTA Section */}
        <section className={styles.ctaSection}>
          <div className="container">
            <h2 className={styles.ctaTitle}>Ready to Build the Future of Robotics?</h2>
            <p className={styles.ctaDescription}>
              Start your journey from foundational concepts to building autonomous humanoid systems
            </p>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro/what-is-physical-ai">
              Begin Learning Now →
            </Link>
          </div>
        </section>
      </main>
    </Layout>
  );
}
