/**
 * Language Switcher Component
 *
 * Toggles between English (/docs/en/) and Urdu (/docs/ur/) documentation.
 * Preserves the current page path when switching languages.
 */

import React from 'react';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import styles from './LangSwitcher.module.css';

export function LangSwitcher(): JSX.Element | null {
  const location = useLocation();
  const { pathname } = location;

  // Only show on docs pages
  if (!pathname.startsWith('/docs/')) {
    return null;
  }

  // Determine current language and target URL
  const isEnglish = pathname.includes('/docs/en/');
  const isUrdu = pathname.includes('/docs/ur/');

  if (!isEnglish && !isUrdu) {
    // If neither, default to showing English link
    return null;
  }

  const targetPath = isEnglish
    ? pathname.replace('/docs/en/', '/docs/ur/')
    : pathname.replace('/docs/ur/', '/docs/en/');

  const targetLanguage = isEnglish ? 'اردو' : 'English';
  const currentLanguage = isEnglish ? 'EN' : 'UR';

  return (
    <div className={styles.langSwitcher}>
      <span className={styles.currentLang}>{currentLanguage}</span>
      <Link
        to={targetPath}
        className={styles.switchButton}
        title={`Switch to ${isEnglish ? 'Urdu' : 'English'}`}
      >
        {targetLanguage}
      </Link>
    </div>
  );
}

export default LangSwitcher;
