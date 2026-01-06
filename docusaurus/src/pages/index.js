import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h1 className="hero__title">{siteConfig.title}</h1>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Start Reading
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Comprehensive curriculum for Physical and AI Humanoid Robot development">
      <HomepageHeader />
      <main>
        <section className={styles.introSection}>
          <div className="container">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2>Learn AI-Powered Robotics</h2>
                <p className={styles.introText}>
                  This comprehensive curriculum teaches you how to develop AI-powered humanoid robots
                  using Isaac Sim, ROS 2, and advanced machine learning techniques. From basic
                  navigation to complex vision-language-action systems, master the complete pipeline
                  of modern robotics development.
                </p>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.modulesSection}>
          <div className="container">
            <div className="row">
              <div className="col col--10 col--offset-1">
                <div className="row">
                  <div className="col col--3">
                    <div className="card">
                      <div className="card__header">
                        <h3>Module 1</h3>
                      </div>
                      <div className="card__body">
                        <p>Introduction to Isaac Sim and basic robot simulation</p>
                      </div>
                    </div>
                  </div>
                  <div className="col col--3">
                    <div className="card">
                      <div className="card__header">
                        <h3>Module 2</h3>
                      </div>
                      <div className="card__body">
                        <p>ROS 2 fundamentals and robot communication</p>
                      </div>
                    </div>
                  </div>
                  <div className="col col--3">
                    <div className="card">
                      <div className="card__header">
                        <h3>Module 3</h3>
                      </div>
                      <div className="card__body">
                        <p>AI perception and navigation systems</p>
                      </div>
                    </div>
                  </div>
                  <div className="col col--3">
                    <div className="card">
                      <div className="card__header">
                        <h3>Module 4</h3>
                      </div>
                      <div className="card__body">
                        <p>Vision-Language-Action systems for voice control</p>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}