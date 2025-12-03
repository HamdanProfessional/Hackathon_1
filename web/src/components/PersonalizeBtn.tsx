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
import { useSession } from '../lib/auth-client';
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
  const [skillLevel, setSkillLevel] = useState<string>('Beginner');
  const location = useLocation();
  const { data: session } = useSession();

  // Reset personalization state when navigating to a different chapter
  useEffect(() => {
    setIsPersonalized(false);
    setError(null);
    setOriginalContent('');
  }, [location.pathname]);

  // Get user hardware background and skill level from session or fallback to localStorage
  useEffect(() => {
    console.log('🔍 PersonalizeBtn - Session data:', {
      session,
      user: session?.user,
      // @ts-ignore
      hardware_bg: session?.user?.hardware_bg,
      // @ts-ignore
      skill_level: session?.user?.skill_level,
    });

    // Better-Auth stores custom fields with snake_case
    // @ts-ignore - Better-Auth custom fields
    if (session?.user?.hardware_bg) {
      // @ts-ignore
      const hw = session.user.hardware_bg;
      console.log('✅ Setting hardware from session:', hw);
      setHardwareBg(hw);
    } else {
      const bg = localStorage.getItem('user_hardware_bg') || 'Laptop CPU';
      console.log('⚠️ No hardware in session, using fallback:', bg);
      setHardwareBg(bg);
    }

    // @ts-ignore - Better-Auth custom fields
    if (session?.user?.skill_level) {
      // @ts-ignore
      const skill = session.user.skill_level;
      console.log('✅ Setting skill level from session:', skill);
      setSkillLevel(skill);
    } else {
      const level = localStorage.getItem('user_skill_level') || 'Beginner';
      console.log('⚠️ No skill level in session, using fallback:', level);
      setSkillLevel(level);
    }
  }, [session]);

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
      tempDiv.setAttribute('data-personalized', 'true');

      // Convert markdown-like formatting to HTML
      const htmlContent = convertMarkdownToHTML(newContent);
      tempDiv.innerHTML = `<div class="${styles.contentWrapper}">${htmlContent}</div>`;

      // Find the main content area wrapper more precisely
      // Look for the actual article content container
      const mainContentDiv = contentElement.querySelector('.theme-doc-markdown') ||
                              contentElement.querySelector('article > div') ||
                              contentElement;

      // Find the heading
      const heading = mainContentDiv.querySelector('h1, h2');

      // More precise selection: only hide direct content children, not structural elements
      // Avoid hiding navigation, TOC, or other layout components
      const elementsToHide = mainContentDiv.querySelectorAll(':scope > p, :scope > ul, :scope > ol, :scope > pre, :scope > blockquote, :scope > h3, :scope > h4, :scope > h5, :scope > h6, :scope > div:not([class*="theme-"]):not([class*="docusaurus"]):not([data-personalized])');

      elementsToHide.forEach((el) => {
        // Get the computed display value before hiding
        const computedStyle = window.getComputedStyle(el as HTMLElement);
        const originalDisplay = computedStyle.display;

        (el as HTMLElement).setAttribute('data-original-display', originalDisplay);
        (el as HTMLElement).style.display = 'none';
      });

      // Insert personalized content after heading or at the start
      if (heading && heading.parentNode) {
        heading.parentNode.insertBefore(tempDiv, heading.nextSibling);
      } else {
        mainContentDiv.insertBefore(tempDiv, mainContentDiv.firstChild);
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
      const personalizedWrapper = contentElement.querySelector('[data-personalized="true"]');
      if (personalizedWrapper) {
        personalizedWrapper.remove();
      }

      // Show hidden original content - search more broadly in case elements moved
      const hiddenElements = document.querySelectorAll('[data-original-display]');
      hiddenElements.forEach((el) => {
        const originalDisplay = (el as HTMLElement).getAttribute('data-original-display') || 'block';
        (el as HTMLElement).style.display = originalDisplay === 'none' ? 'block' : originalDisplay;
        (el as HTMLElement).removeAttribute('data-original-display');
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
      // Request personalized content from API with skill level
      const response = await apiClient.personalize(content, hardwareBg, skillLevel);

      if (!response.data || response.status !== 200) {
        throw new Error(response.error || 'Failed to personalize content');
      }

      // Get personalized content
      const personalizedContent = response.data.personalized_content || response.data.response;

      // Validate we got actual content back
      if (!personalizedContent || personalizedContent.trim().length === 0) {
        throw new Error('Received empty response from server');
      }

      // Check if the response is an error message
      if (personalizedContent.startsWith('Unable to personalize')) {
        throw new Error(personalizedContent.replace('Unable to personalize content: ', ''));
      }

      // Only replace content in DOM if we successfully got valid content
      replaceContentInDOM(personalizedContent);

      setIsPersonalized(true);
      if (onPersonalize) {
        onPersonalize(personalizedContent);
      }
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Unknown error occurred';
      setError(`Personalization failed: ${errorMsg}`);
      console.error('Personalization error:', err);

      // IMPORTANT: Don't leave content hidden if there was an error
      // Content is still in original state, so just clear the loading state
    } finally {
      setIsLoading(false);
    }
  };

  // Don't render if user is not logged in
  if (!session?.user) {
    return null;
  }

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
        title={isPersonalized ? 'Show original content' : `Personalize for ${hardwareBg} (${skillLevel})`}
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
            <span className={styles.icon}>🔧</span> Personalize for Me
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
 * Convert markdown-like content to HTML
 * Handles basic markdown syntax for rendering personalized content
 */
function convertMarkdownToHTML(markdown: string): string {
  let html = markdown;

  // Escape dangerous HTML to prevent XSS
  const dangerousPatterns = /<script|<iframe|<object|<embed|javascript:/gi;
  if (dangerousPatterns.test(html)) {
    console.warn('Potentially dangerous HTML detected, sanitizing...');
    html = html.replace(dangerousPatterns, '');
  }

  // Convert code blocks (```language ... ```)
  html = html.replace(/```(\w+)?\n([\s\S]*?)```/g, (_, lang, code) => {
    return `<pre><code class="language-${lang || 'text'}">${escapeCodeBlock(code.trim())}</code></pre>`;
  });

  // Convert inline code (`code`)
  html = html.replace(/`([^`]+)`/g, '<code>$1</code>');

  // Convert headers
  html = html.replace(/^### (.*$)/gim, '<h3>$1</h3>');
  html = html.replace(/^## (.*$)/gim, '<h2>$1</h2>');
  html = html.replace(/^# (.*$)/gim, '<h1>$1</h1>');

  // Convert bold (**text** or __text__)
  html = html.replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>');
  html = html.replace(/__(.+?)__/g, '<strong>$1</strong>');

  // Convert italic (*text* or _text_)
  html = html.replace(/\*(.+?)\*/g, '<em>$1</em>');
  html = html.replace(/_(.+?)_/g, '<em>$1</em>');

  // Convert unordered lists
  html = html.replace(/^\* (.+)$/gim, '<li>$1</li>');
  html = html.replace(/^- (.+)$/gim, '<li>$1</li>');
  html = html.replace(/(<li>.*<\/li>)/s, '<ul>$1</ul>');

  // Convert ordered lists
  html = html.replace(/^\d+\. (.+)$/gim, '<li>$1</li>');

  // Convert line breaks to paragraphs
  html = html.split('\n\n').map(para => {
    if (!para.trim()) return '';
    if (para.startsWith('<')) return para; // Already HTML
    return `<p>${para.trim()}</p>`;
  }).join('\n');

  // Convert single line breaks to <br>
  html = html.replace(/\n/g, '<br>');

  return html;
}

/**
 * Escape code blocks to prevent HTML injection
 */
function escapeCodeBlock(code: string): string {
  const map: Record<string, string> = {
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
  };
  return code.replace(/[&<>]/g, (char) => map[char]);
}

export default PersonalizeBtn;
