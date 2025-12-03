/**
 * API Client Utility
 *
 * Handles requests to the Python API (localhost:8000) and Auth API (localhost:3001).
 * Includes interceptors for session token management and error handling.
 */

interface RequestOptions {
  method?: 'GET' | 'POST' | 'PUT' | 'DELETE' | 'PATCH';
  headers?: Record<string, string>;
  body?: any;
  includeToken?: boolean;
}

interface ApiResponse<T = any> {
  status: number;
  data: T;
  error?: string;
}

class ApiClient {
  private pythonBaseUrl: string;
  private authBaseUrl: string;
  private tokenKey = 'session_token';

  constructor(pythonBaseUrl = 'http://localhost:8000', authBaseUrl = 'http://localhost:3001') {
    this.pythonBaseUrl = pythonBaseUrl;
    this.authBaseUrl = authBaseUrl;
  }

  /**
   * Get the stored session token from localStorage
   */
  private getToken(): string | null {
    if (typeof window === 'undefined') return null;
    return localStorage.getItem(this.tokenKey);
  }

  /**
   * Store session token in localStorage
   */
  setToken(token: string): void {
    if (typeof window !== 'undefined') {
      localStorage.setItem(this.tokenKey, token);
    }
  }

  /**
   * Remove session token from localStorage
   */
  clearToken(): void {
    if (typeof window !== 'undefined') {
      localStorage.removeItem(this.tokenKey);
    }
  }

  /**
   * Internal fetch wrapper with interceptors
   */
  private async fetchWithInterceptors(
    url: string,
    options: RequestOptions = {}
  ): Promise<Response> {
    const {
      method = 'GET',
      headers = {},
      body,
      includeToken = true,
    } = options;

    // Prepare request headers
    const requestHeaders: Record<string, string> = {
      'Content-Type': 'application/json',
      ...headers,
    };

    // Attach session token if available and requested
    if (includeToken) {
      const token = this.getToken();
      if (token) {
        requestHeaders['Authorization'] = `Bearer ${token}`;
      }
    }

    // Prepare request config
    const config: RequestInit = {
      method,
      headers: requestHeaders,
    };

    if (body && (method === 'POST' || method === 'PUT' || method === 'PATCH')) {
      config.body = JSON.stringify(body);
    }

    // Make the request
    const response = await fetch(url, config);

    // Handle authentication errors
    if (response.status === 401) {
      this.clearToken();
      // Optionally redirect to login
      if (typeof window !== 'undefined' && window.location.pathname !== '/login') {
        // window.location.href = '/login';
      }
    }

    return response;
  }

  /**
   * GET request to Python API
   */
  async getApi<T = any>(endpoint: string, options?: RequestOptions): Promise<ApiResponse<T>> {
    try {
      const url = `${this.pythonBaseUrl}${endpoint}`;
      const response = await this.fetchWithInterceptors(url, {
        ...options,
        method: 'GET',
      });

      const data = await response.json();

      return {
        status: response.status,
        data: response.ok ? data : null,
        error: response.ok ? undefined : data?.error || response.statusText,
      };
    } catch (error) {
      return {
        status: 500,
        data: null,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  /**
   * POST request to Python API
   */
  async postApi<T = any>(
    endpoint: string,
    body?: any,
    options?: RequestOptions
  ): Promise<ApiResponse<T>> {
    try {
      const url = `${this.pythonBaseUrl}${endpoint}`;
      const response = await this.fetchWithInterceptors(url, {
        ...options,
        method: 'POST',
        body,
      });

      const data = await response.json();

      return {
        status: response.status,
        data: response.ok ? data : null,
        error: response.ok ? undefined : data?.error || response.statusText,
      };
    } catch (error) {
      return {
        status: 500,
        data: null,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  /**
   * PUT request to Python API
   */
  async putApi<T = any>(
    endpoint: string,
    body?: any,
    options?: RequestOptions
  ): Promise<ApiResponse<T>> {
    try {
      const url = `${this.pythonBaseUrl}${endpoint}`;
      const response = await this.fetchWithInterceptors(url, {
        ...options,
        method: 'PUT',
        body,
      });

      const data = await response.json();

      return {
        status: response.status,
        data: response.ok ? data : null,
        error: response.ok ? undefined : data?.error || response.statusText,
      };
    } catch (error) {
      return {
        status: 500,
        data: null,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  /**
   * DELETE request to Python API
   */
  async deleteApi<T = any>(endpoint: string, options?: RequestOptions): Promise<ApiResponse<T>> {
    try {
      const url = `${this.pythonBaseUrl}${endpoint}`;
      const response = await this.fetchWithInterceptors(url, {
        ...options,
        method: 'DELETE',
      });

      const data = response.status !== 204 ? await response.json() : null;

      return {
        status: response.status,
        data: response.ok ? data : null,
        error: response.ok ? undefined : data?.error || response.statusText,
      };
    } catch (error) {
      return {
        status: 500,
        data: null,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  /**
   * GET request to Auth API
   */
  async getAuth<T = any>(endpoint: string, options?: RequestOptions): Promise<ApiResponse<T>> {
    try {
      const url = `${this.authBaseUrl}${endpoint}`;
      const response = await this.fetchWithInterceptors(url, {
        ...options,
        method: 'GET',
      });

      const data = await response.json();

      return {
        status: response.status,
        data: response.ok ? data : null,
        error: response.ok ? undefined : data?.error || response.statusText,
      };
    } catch (error) {
      return {
        status: 500,
        data: null,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  /**
   * POST request to Auth API
   */
  async postAuth<T = any>(
    endpoint: string,
    body?: any,
    options?: RequestOptions
  ): Promise<ApiResponse<T>> {
    try {
      const url = `${this.authBaseUrl}${endpoint}`;
      const response = await this.fetchWithInterceptors(url, {
        ...options,
        method: 'POST',
        body,
      });

      const data = await response.json();

      return {
        status: response.status,
        data: response.ok ? data : null,
        error: response.ok ? undefined : data?.error || response.statusText,
      };
    } catch (error) {
      return {
        status: 500,
        data: null,
        error: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  /**
   * Convenience method for chat API
   */
  async chat(message: string, history?: Array<any>): Promise<ApiResponse<any>> {
    return this.postApi('/chat', { message, history });
  }

  /**
   * Convenience method for personalize API
   */
  async personalize(content: string, hardwareBg: string, skillLevel: string = 'Beginner'): Promise<ApiResponse<any>> {
    return this.postApi('/personalize', { content, hardware_bg: hardwareBg, skill_level: skillLevel });
  }

  /**
   * Convenience method for health check
   */
  async healthCheck(): Promise<ApiResponse<any>> {
    return this.getApi('/api/health');
  }
}

// Get backend URLs from environment variables or use defaults
const getBackendUrls = () => {
  // For Docusaurus, we can use customFields in config or browser globals
  if (typeof window !== 'undefined' && (window as any).docusaurus?.siteConfig?.customFields) {
    const customFields = (window as any).docusaurus.siteConfig.customFields;
    return {
      apiUrl: customFields.apiUrl || 'http://localhost:8000',
      authUrl: customFields.authUrl || 'http://localhost:3001',
    };
  }

  // Fallback to localhost for local development
  return {
    apiUrl: 'http://localhost:8000',
    authUrl: 'http://localhost:3001',
  };
};

const { apiUrl, authUrl } = getBackendUrls();

// Export singleton instance with environment-based URLs
export const apiClient = new ApiClient(apiUrl, authUrl);

// Export class for testing/customization
export default ApiClient;
