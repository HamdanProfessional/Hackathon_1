/**
 * Language Toggle Component
 *
 * Switches between English and Urdu documentation
 * Uses folder-based routing: /docs/en/ and /docs/ur/
 */

import React from 'react';
import { useLocation } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function LanguageToggle() {
  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl;

  // Determine current language from pathname
  const isUrdu = location.pathname.includes('/ur/');
  const currentLang = isUrdu ? 'ur' : 'en';
  const targetLang = isUrdu ? 'en' : 'ur';

  // Convert current path to target language
  const getTargetPath = () => {
    const pathname = location.pathname;

    // If we're on a doc page, swap the language folder
    if (pathname.includes('/docs/en/')) {
      return pathname.replace('/docs/en/', '/docs/ur/');
    } else if (pathname.includes('/docs/ur/')) {
      return pathname.replace('/docs/ur/', '/docs/en/');
    }

    // Default: go to intro page of target language
    return `${baseUrl}docs/${targetLang}/intro`;
  };

  return (
    <div className="navbar__item" style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
      <Link
        to={getTargetPath()}
        className="button button--outline button--sm"
        style={{
          display: 'flex',
          alignItems: 'center',
          gap: '6px',
          fontWeight: 500,
          minWidth: '80px',
          justifyContent: 'center',
        }}
      >
        <span style={{ fontSize: '16px' }}>
          {targetLang === 'ur' ? '🇵🇰' : '🇬🇧'}
        </span>
        <span>
          {targetLang === 'ur' ? 'اردو' : 'English'}
        </span>
      </Link>
    </div>
  );
}
