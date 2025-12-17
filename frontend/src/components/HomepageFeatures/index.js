import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    id: 1,
    title: 'Module 1: Robotic Nervous System with ROS',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Master the fundamentals of ROS 2, the communication backbone of modern robotics.
        Build distributed systems.
      </>
    ),
    tags: ['ROS 2', 'DDS', 'Microservices'],
  },
  {
    id: 2,
    title: 'Module 2: Digital Twin & Simulation',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Create realistic virtual environments using Gazebo & RViz. Test and validate
        your robotic systems in safe, virtual environments.
      </>
    ),
    tags: ['Gazebo', 'RViz', 'Unity3D'],
  },
  {
    id: 3,
    title: 'Module 3: AI Robot Brain Development',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Develop perception systems using NVIDIA Isaac ROS. Build AI-powered robots that
        see, understand, and navigate.
      </>
    ),
    tags: ['Computer Vision', 'MLOps'],
  },
];

function Feature({ id, Svg, title, description, tags }) {
  return (
    <div className={clsx('col col--4', styles.featureCard)} data-id={`feature-${id}`}>
      <div className={styles.featureNumber}>{`0${id}`}</div>
      
      <div className={styles.featureIconWrapper}>
        <div className={styles.iconBackground}></div>
        <Svg className={styles.featureSvg} role="img" />
      </div>
      
      <div className={styles.featureContent}>
        <Heading as="h3" className={styles.featureTitle}>
          {title}
        </Heading>
        <p className={styles.featureDescription}>{description}</p>
        
        <div className={styles.featureTags}>
          {tags.map((tag, index) => (
            <span key={index} className={styles.tag}>
              {tag}
            </span>
          ))}
        </div>
        
        <div className={styles.featureCta}>
          <span className={styles.ctaText}>Explore Module</span>
          <svg className={styles.ctaArrow} width="16" height="16" viewBox="0 0 24 24" fill="none">
            <path d="M5 12H19M19 12L12 5M19 12L12 19" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </div>
      </div>
      
      <div className={styles.cardGlow}></div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Comprehensive Robotics Curriculum
          </Heading>
          <p className={styles.sectionSubtitle}>
            Master modern robotics through our structured three-module approach, 
            from foundational systems to advanced AI integration
          </p>
        </div>
        
        <div className="row">
          {FeatureList.map((props) => (
            <Feature key={props.id} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}