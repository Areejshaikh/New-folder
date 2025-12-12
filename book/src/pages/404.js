import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './styles.module.css';

function NotFound() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={`Page Not Found - ${siteConfig.title}`}>
      <main className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1 className="hero__title">Page Not Found</h1>
            <p className="hero__subtitle">
              We couldn't find the page you were looking for.
            </p>
            <div className="margin-vert--lg">
              <Link className="button button--primary button--lg" to="/">
                Back to Home
              </Link>
              <Link className="button button--secondary button--lg margin-left--md" to="/docs/intro">
                Book Introduction
              </Link>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default NotFound;