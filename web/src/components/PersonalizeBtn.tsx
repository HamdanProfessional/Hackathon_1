/**
 * PersonalizeButton Component
 *
 * Displays a button that allows users to personalize chapter content based on their hardware background.
 * On click, extracts markdown content from the DOM and requests personalized rewriting from the API.
 *
 * Features:
 * - Shows hardware-specific label (e.g., "Personalize for RTX4090")
 * - Toggles between original and personalized content
 * - Streaming LLM response with loading indicator
 * - Error handling with user feedback
 * - Session-based state (resets on navigation)
 */

import React, { useState, useContext, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import { apiClient } from '../utils/api';
import styles from './PersonalizeBtn.module.css';

interface PersonalizeBtnProps {
  onPersonalize?: (content: string) => void;
}

export const PersonalizeBtn: React.FC<PersonalizeBtnProps> = ({ onPersonalize }) => {
  const [isLoading, setIsLoading] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [originalContent, setOriginalContent] = useState<string>('');
  const [hardwareBg, setHardwareBg] = useState<string>('');
  const location = useLocation();

  // Reset personalization state when navigating to a different chapter
  useEffect(() => {
    setIsPersonalized(false);
    setError(null);
    setOriginalContent('');
  }, [location.pathname]);

  // Get user hardware background from localStorage or context
  useEffect(() => {
    const bg = localStorage.getItem('user_hardware_bg') || 'Unknown';
    setHardwareBg(bg);
  }, []);

  /**
   * Extract markdown content from the main article element
   * Targets Docusaurus DocItem structure
   */
  const extractContentFromDOM = (): string | null => {
    try {
      // Try multiple selectors for Docusaurus article content
      const selectors = [
        'article',
        '[role="main"]',
        '.docMainContainer',
        '.markdown',
      ];

      let contentElement: HTMLElement | null = null;
      for (const selector of selectors) {
        contentElement = document.querySelector(selector);
        if (contentElement) break;
      }

      if (!contentElement) {
        console.warn('Could not find article content element');
        return null;
      }

      // Clone to avoid modifying DOM
      const clone = contentElement.cloneNode(true) as HTMLElement;

      // Remove interactive elements and navigation
      const elementsToRemove = clone.querySelectorAll(
        'button, nav, .toc, .docusaurus-mt-lg, .pagination'
      );
      elementsToRemove.forEach((el) => el.remove());

      // Get text content
      return clone.innerText || clone.textContent || '';
    } catch (err) {
      console.error('Error extracting content from DOM:', err);
      return null;
    }
  };

  /**
   * Replace DOM content with new personalized text
   */
  const replaceContentInDOM = (newContent: string): void => {
    try {
      const selectors = [
        'article',
        '[role="main"]',
        '.docMainContainer',
        '.markdown',
      ];

      let contentElement: HTMLElement | null = null;
      for (const selector of selectors) {
        contentElement = document.querySelector(selector);
        if (contentElement) break;
      }

      if (!contentElement) {
        console.warn('Could not find article content element to update');
        return;
      }

      // Create a container for the new content
      const tempDiv = document.createElement('div');
      tempDiv.className = styles.personalizedContent;
      tempDiv.innerHTML = `<div class="${styles.contentWrapper}">${escapeHtml(newContent)}</div>`;

      // Replace content (but preserve the first heading)
      const heading = contentElement.querySelector('h1, h2');
      if (heading) {
        heading.parentNode?.insertBefore(tempDiv, heading.nextSibling);
        // Hide original content
        const originalText = contentElement.querySelectorAll('p, li, code');
        originalText.forEach((el) => {
          if (el !== heading) {
            (el as HTMLElement).style.display = 'none';
          }
        });
      } else {
        contentElement.innerHTML = tempDiv.innerHTML;
      }
    } catch (err) {
      console.error('Error updating DOM with personalized content:', err);
    }
  };

  /**
   * Restore original content in DOM
   */
  const restoreOriginalContent = (): void => {
    try {
      const selectors = [
        'article',
        '[role="main"]',
        '.docMainContainer',
        '.markdown',
      ];

      let contentElement: HTMLElement | null = null;
      for (const selector of selectors) {
        contentElement = document.querySelector(selector);
        if (contentElement) break;
      }

      if (!contentElement) return;

      // Remove personalized content wrapper
      const personalizedWrapper = contentElement.querySelector(`.${styles.personalizedContent}`);
      if (personalizedWrapper) {
        personalizedWrapper.remove();
      }

      // Show hidden original content
      const hiddenElements = contentElement.querySelectorAll('[style*="display: none"]');
      hiddenElements.forEach((el) => {
        (el as HTMLElement).style.display = '';
      });
    } catch (err) {
      console.error('Error restoring original content:', err);
    }
  };

  /**
   * Handle personalize button click
   */
  const handlePersonalize = async (): Promise<void> => {
    // If already personalized, toggle back to original
    if (isPersonalized) {
      restoreOriginalContent();
      setIsPersonalized(false);
      return;
    }

    // Extract current content
    const content = extractContentFromDOM();
    if (!content) {
      setError('Could not extract chapter content. Please try again.');
      return;
    }

    // Store original content for later restoration
    setOriginalContent(content);
    setIsLoading(true);
    setError(null);

    try {
      // Request personalized content from API
      const response = await apiClient.personalize(content, hardwareBg);

      if (!response.data || response.status !== 200) {
        throw new Error(response.error || 'Failed to personalize content');
      }

      // Replace content in DOM
      const personalizedContent = response.data.personalized_content || response.data.response;
      replaceContentInDOM(personalizedContent);

      setIsPersonalized(true);
      if (onPersonalize) {
        onPersonalize(personalizedContent);
      }
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Unknown error occurred';
      setError(`Personalization failed: ${errorMsg}`);
      console.error('Personalization error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  // Don't render if no hardware background is set
  if (!hardwareBg || hardwareBg === 'Unknown') {
    return null;
  }

  return (
    <div className={styles.personalizeContainer}>
      <button
        className={`${styles.personalizeBtn} ${isPersonalized ? styles.active : ''}`}
        onClick={handlePersonalize}
        disabled={isLoading}
        title={isPersonalized ? 'Show original content' : `Personalize for ${hardwareBg}`}
      >
        {isLoading ? (
          <span className={styles.loading}>
            <span className={styles.spinner}>⏳</span> Personalizing...
          </span>
        ) : isPersonalized ? (
          <span>
            <span className={styles.icon}>✓</span> View Original
          </span>
        ) : (
          <span>
            <span className={styles.icon}>🔧</span> Personalize for {hardwareBg}
          </span>
        )}
      </button>

      {error && (
        <div className={styles.error}>
          <span className={styles.errorIcon}>⚠️</span>
          {error}
          <button
            className={styles.dismissError}
            onClick={() => setError(null)}
            aria-label="Dismiss error"
          >
            ✕
          </button>
        </div>
      )}
    </div>
  );
};

/**
 * Simple HTML escape utility
 */
function escapeHtml(text: string): string {
  const map: Record<string, string> = {
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    '"': '&quot;',
    "'": '&#039;',
  };
  return text.replace(/[&<>"']/g, (char) => map[char]);
}

export default PersonalizeBtn;
