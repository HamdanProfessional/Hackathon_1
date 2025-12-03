import type { VercelRequest, VercelResponse } from '@vercel/node';

/**
 * Vercel Serverless Function for Authentication
 *
 * This function handles all authentication requests in a serverless environment
 */
export default async function handler(req: VercelRequest, res: VercelResponse) {
  // Enable CORS
  const origin = req.headers.origin || '';

  if (origin.includes('localhost') || origin.endsWith('.vercel.app')) {
    res.setHeader('Access-Control-Allow-Origin', origin);
  } else {
    res.setHeader('Access-Control-Allow-Origin', '*');
  }

  res.setHeader('Access-Control-Allow-Credentials', 'true');
  res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization');

  // Handle preflight
  if (req.method === 'OPTIONS') {
    res.status(200).end();
    return;
  }

  try {
    const pathname = req.url || '';

    // Health check endpoint
    if (pathname === '/api/auth/health' || pathname === '/health') {
      res.status(200).json({
        status: 'healthy',
        service: 'auth',
        version: '1.0.0',
        timestamp: new Date().toISOString(),
      });
      return;
    }

    // Root endpoint
    if (pathname === '/api/auth' && req.method === 'GET') {
      res.status(200).json({
        message: 'Physical AI Textbook Auth Service',
        docs: '/api/auth/docs',
      });
      return;
    }

    // Route to better-auth handler (lazy load)
    if (pathname.startsWith('/api/auth')) {
      // Lazy load auth config only when needed
      const { auth } = await import('../src/auth.config');

      // Get request body
      const body = req.method === 'POST' || req.method === 'PUT'
        ? JSON.stringify(req.body)
        : undefined;

      // Convert Vercel request to Web API Request
      const url = `https://${req.headers.host}${req.url}`;
      const request = new Request(url, {
        method: req.method as string,
        headers: req.headers as any,
        body: body,
      });

      const response = await auth.handler(request);

      if (response) {
        // Convert Web API Response to Vercel response
        const headers: Record<string, string> = {};
        response.headers.forEach((value, key) => {
          headers[key] = value;
        });

        // Set headers
        Object.entries(headers).forEach(([key, value]) => {
          res.setHeader(key, value);
        });

        // Get response body
        const responseText = await response.text();

        res.status(response.status).send(responseText);
        return;
      }
    }

    // 404
    res.status(404).json({ error: 'Not Found' });
  } catch (error) {
    console.error('Error handling request:', error);
    res.status(500).json({
      error: 'Internal Server Error',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
}
