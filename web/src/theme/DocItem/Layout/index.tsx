/**
 * DocItem Layout Wrapper
 *
 * Custom Docusaurus theme layout that integrates:
 * - PersonalizeButton: Hardware-specific content adaptation
 * - ChatWidget: RAG-powered Q&A assistant
 * - LangSwitch: Bilingual language toggling (to be added in future phase)
 *
 * This wrapper extends the default Docusaurus DocItem/Layout with custom UI components
 * positioned strategically throughout the document viewing experience.
 */

import React from 'react';
import { useLocation } from '@docusaurus/router';
import Layout from '@theme/DocItem/Layout';
import { PersonalizeBtn } from '../../../components/PersonalizeBtn';
import { ChatWidget } from '../../../components/ChatWidget';
import styles from './Layout.module.css';

interface DocItemLayoutProps {
  children?: React.ReactNode;
}

/**
 * Custom DocItem Layout wrapper
 *
 * Injects custom components into the Docusaurus layout:
 * 1. PersonalizeButton - positioned below the main heading
 * 2. ChatWidget - floating widget (bottom-right)
 * 3. LangSwitch - (future phase) positioned top-right
 */
export default function DocItemLayoutWrapper(props: DocItemLayoutProps): React.ReactElement {
  const location = useLocation();

  return (
    <Layout {...props}>
      <div className={styles.docItemWrapper}>
        {/* Personalize Button - positioned at the top of content */}
        <div className={styles.personalizeSection}>
          <PersonalizeBtn />
        </div>

        {/* Main content */}
        <div className={styles.mainContent}>
          {props.children}
        </div>

        {/* Language Switch - placeholder for future phase */}
        {/* Will be added in Phase 6 (User Story 4) */}
        {/* <LangSwitch /> */}

        {/* Chat Widget - floating globally */}
        <ChatWidget />
      </div>
    </Layout>
  );
}
