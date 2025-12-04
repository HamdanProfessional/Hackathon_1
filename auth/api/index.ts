import type { VercelRequest, VercelResponse } from '@vercel/node';
import { toNodeHandler } from 'better-auth/node';

/**
 * Vercel Serverless Function for Authentication
 *
 * This function handles all authentication requests in a serverless environment
 * Uses better-auth's toNodeHandler for proper request handling
 */

// Lazy load auth instance
let authHandler: any = null;

async function getAuthHandler() {
  if (!authHandler) {
    const { auth } = await import('../src/auth.config.js');
    authHandler = toNodeHandler(auth);
  }
  return authHandler;
}

export default async function handler(req: VercelRequest, res: VercelResponse) {
  // Enable CORS
  const origin = req.headers.origin || '';

  // Allow GitHub Pages, localhost, and Vercel deployments
  if (origin.includes('localhost') ||
      origin.endsWith('.vercel.app') ||
      origin.endsWith('.github.io')) {
    res.setHeader('Access-Control-Allow-Origin', origin);
    res.setHeader('Access-Control-Allow-Credentials', 'true');
  } else {
    res.setHeader('Access-Control-Allow-Origin', '*');
  }

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

    // Route to better-auth handler using Node.js adapter
    if (pathname.startsWith('/api/auth')) {
      // Don't log or access req.body - let toNodeHandler handle it
      console.log(`📥 Auth request: ${req.method} ${pathname}`);

      const handler = await getAuthHandler();

      // Intercept the response to log it
      const originalEnd = res.end;
      const originalSend = res.send;
      const originalJson = res.json;

      let responseBody: any = null;

      res.send = function(body: any) {
        responseBody = body;
        console.log(`📤 Auth response status: ${res.statusCode}`);
        console.log(`📤 Auth response body:`, typeof body === 'string' ? body : JSON.stringify(body));
        return originalSend.call(this, body);
      };

      res.json = function(body: any) {
        responseBody = body;
        console.log(`📤 Auth response status: ${res.statusCode}`);
        console.log(`📤 Auth response body:`, JSON.stringify(body));
        return originalJson.call(this, body);
      };

      return handler(req, res);
    }

    // 404
    res.status(404).json({ error: 'Not Found' });
  } catch (error) {
    console.error('Error handling request:', error);
    res.status(500).json({
      error: 'Internal Server Error',
      message: error instanceof Error ? error.message : 'Unknown error',
      stack: process.env.NODE_ENV === 'development' ? (error as Error).stack : undefined
    });
  }
}
