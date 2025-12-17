import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import FloatingChatbot from '@site/src/components/FloatingChatbot';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      {/* Animated background elements */}
      <div className={styles.animatedBackground}>
        <div className={styles.floatingOrbs}>
          {[...Array(5)].map((_, i) => (
            <div key={i} className={styles.floatingOrb} style={{
              '--delay': `${i * 2}s`,
              '--size': `${40 + i * 10}px`,
              '--opacity': `${0.2 + i * 0.05}`
            }}></div>
          ))}
        </div>
        
        <div className={styles.gridLines}>
          <div className={styles.gridLine}></div>
          <div className={styles.gridLine}></div>
          <div className={styles.gridLine}></div>
        </div>
      </div>
      
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroTextContainer}>
            <div className={styles.badgeContainer}>
              <span className={styles.heroBadge}>Advanced Robotics Curriculum</span>
            </div>
            
            <Heading as="h1" className={clsx('hero__title', styles.hero__title)}>
              <span className={styles.titleGradient}>{siteConfig.title}</span>
            </Heading>
            
            <p className={clsx('hero__subtitle', styles.hero__subtitle)}>
              <span className={styles.highlightText}>Bridging the gap</span> between digital intelligence 
              and physical autonomy in next-generation robotics
            </p>
          </div>
          
          <div className={styles.buttons}>
            <Link
              className={clsx('button', styles.primaryButton)}
              to="/docs/module-1-robotic-nervous-system/ros2-fundamentals">
              <span className={styles.buttonText}>Start Learning Journey</span>
              <svg className={styles.buttonIcon} width="20" height="20" viewBox="0 0 24 24" fill="none">
                <path d="M5 12H19M19 12L12 5M19 12L12 19" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            </Link>
            
            <Link
              className={clsx('button', styles.secondaryButton)}
              to="/docs/intro">
              <span className={styles.buttonText}>View Curriculum</span>
              <svg className={styles.buttonIcon} width="20" height="20" viewBox="0 0 24 24" fill="none">
                <path d="M9 5H7C5.89543 5 5 5.89543 5 7V19C5 20.1046 5.89543 21 7 21H17C18.1046 21 19 20.1046 19 19V7C19 5.89543 18.1046 5 17 5H15M9 5C9 6.10457 9.89543 7 11 7H13C14.1046 7 15 6.10457 15 5M9 5C9 3.89543 9.89543 3 11 3H13C14.1046 3 15 3.89543 15 5M12 12H15M12 16H15M9 12H9.01M9 16H9.01" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
              </svg>
            </Link>
          </div>
        </div>
      </div>
      
      {/* Scroll indicator */}
      <div className={styles.scrollIndicator}>
        <div className={styles.mouse}>
          <div className={styles.wheel}></div>
        </div>
        <span className={styles.scrollText}>Scroll to explore</span>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title} - Advanced Robotics Platform`}
      description="Master modern robotics with our comprehensive curriculum covering ROS 2, simulation, and AI-powered perception systems.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
      <FloatingChatbot />
    </Layout>
  );
}