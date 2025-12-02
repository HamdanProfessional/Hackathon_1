/**
 * DocItem Layout Wrapper
 *
 * Custom Docusaurus theme layout that integrates:
 * - PersonalizeButton: Hardware-specific content adaptation
 * - ChatWidget: RAG-powered Q&A assistant
 * - Select-to-Ask: Highlight text to add to chat context
 * - LangSwitch: Bilingual language toggling (to be added in future phase)
 *
 * This wrapper extends the default Docusaurus DocItem/Layout with custom UI components
 * positioned strategically throughout the document viewing experience.
 */

import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import Layout from '@theme-original/DocItem/Layout';
import { PersonalizeBtn } from '../../../components/PersonalizeBtn';
import { ChatWidget } from '../../../components/ChatWidget';
import { useChatContext } from '../../../context/ChatContext';
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
 * 3. Select-to-Ask tooltip - appears on text selection
 * 4. LangSwitch - (future phase) positioned top-right
 */
export default function DocItemLayoutWrapper(props: DocItemLayoutProps): React.ReactElement {
  const location = useLocation();
  const { addContext, setIsChatOpen } = useChatContext();
  const [tooltip, setTooltip] = useState<{
    visible: boolean;
    x: number;
    y: number;
    text: string;
  }>({
    visible: false,
    x: 0,
    y: 0,
    text: '',
  });
  const contentRef = useRef<HTMLDivElement>(null);

  const handleMouseUp = () => {
    if (typeof window === 'undefined') return;

    const selection = window.getSelection();
    const selectedText = selection?.toString().trim();

    if (selectedText && selectedText.length > 0) {
      const range = selection?.getRangeAt(0);
      const rect = range?.getBoundingClientRect();

      if (rect) {
        setTooltip({
          visible: true,
          x: rect.left + rect.width / 2,
          y: rect.top - 10,
          text: selectedText,
        });
      }
    } else {
      setTooltip((prev) => ({ ...prev, visible: false }));
    }
  };

  const handleAddToChat = () => {
    if (tooltip.text) {
      addContext(tooltip.text);
      setIsChatOpen(true);
      setTooltip({ visible: false, x: 0, y: 0, text: '' });

      // Clear selection
      if (typeof window !== 'undefined') {
        window.getSelection()?.removeAllRanges();
      }
    }
  };

  // Hide tooltip on scroll or click outside
  useEffect(() => {
    const handleScroll = () => {
      setTooltip((prev) => ({ ...prev, visible: false }));
    };

    const handleClick = (e: MouseEvent) => {
      if (contentRef.current && !contentRef.current.contains(e.target as Node)) {
        setTooltip((prev) => ({ ...prev, visible: false }));
      }
    };

    window.addEventListener('scroll', handleScroll, true);
    window.addEventListener('click', handleClick);

    return () => {
      window.removeEventListener('scroll', handleScroll, true);
      window.removeEventListener('click', handleClick);
    };
  }, []);

  return (
    <Layout {...props}>
      <div className={styles.docItemWrapper}>
        {/* Personalize Button - positioned at the top of content */}
        <div className={styles.personalizeSection}>
          <PersonalizeBtn />
        </div>

        {/* Main content with text selection handler */}
        <div
          ref={contentRef}
          className={styles.mainContent}
          onMouseUp={handleMouseUp}
        >
          {props.children}
        </div>

        {/* Select-to-Ask Tooltip */}
        {tooltip.visible && (
          <div
            className={styles.selectionTooltip}
            style={{
              left: `${tooltip.x}px`,
              top: `${tooltip.y}px`,
            }}
          >
            <button
              className={styles.tooltipButton}
              onClick={handleAddToChat}
              title="Add selected text to chat"
            >
              💬 Add to Chat
            </button>
          </div>
        )}

        {/* Language Switch - placeholder for future phase */}
        {/* Will be added in Phase 6 (User Story 4) */}
        {/* <LangSwitch /> */}

        {/* Chat Widget - floating globally */}
        <ChatWidget />
      </div>
    </Layout>
  );
}
