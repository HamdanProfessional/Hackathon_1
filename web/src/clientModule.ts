/**
 * Client Module - Runs only in the browser
 * Sets up global configuration for auth and API URLs
 */

import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

if (ExecutionEnvironment.canUseDOM) {
  // This code only runs in the browser
  const customFields = (window as any).docusaurus?.siteConfig?.customFields;

  // Store URLs in a global object for auth-client to use
  (window as any).__BACKEND_URLS__ = {
    apiUrl: customFields?.apiUrl || 'http://localhost:8000',
    authUrl: customFields?.authUrl || 'http://localhost:3001',
  };

  console.log('📦 Client Module loaded. Backend URLs:', (window as any).__BACKEND_URLS__);
}
