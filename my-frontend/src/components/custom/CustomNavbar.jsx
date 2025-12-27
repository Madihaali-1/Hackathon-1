import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import {translate} from '@docusaurus/Translate';
import styles from './CustomNavbar.module.css';

function CustomNavbar() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <nav
      className={clsx('navbar', 'navbar--light', 'navbar--fixed-top', styles.customNavbar)}>
      <div className="container">
        <div className="navbar__brand">
          <Link className="navbar__title" to={useBaseUrl('/')}>
            <span>{siteConfig.title}</span>
          </Link>
        </div>

        <div className="navbar__items">
          <Link className="navbar__item navbar__link" to={useBaseUrl('/docs')}>
            {translate({id: 'theme.navbar.docs', message: 'Docs'})}
          </Link>
          <Link className="navbar__item navbar__link" to={useBaseUrl('/blog')}>
            {translate({id: 'theme.navbar.blog', message: 'Blog'})}
          </Link>
        </div>

        <div className="navbar__items navbar__items--right">
          <Link
            className="navbar__item navbar__link"
            href={siteConfig.customFields.githubUrl || 'https://github.com/'}>
            GitHub
          </Link>
        </div>
      </div>
    </nav>
  );
}

export default CustomNavbar;