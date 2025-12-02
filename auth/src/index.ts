import { config } from 'dotenv';
import { resolve } from 'path';

// Load .env from parent directory (repository root)
// override: true forces .env to override system environment variables
config({ path: resolve(__dirname, '../../.env'), override: true });

/**
 * Authentication Server
 *
 * Runs the better-auth service on port 3001
 * Provides endpoints for signup, login, and session management
 */

// Debug: Check if DATABASE_URL is loaded
console.log("🔍 Checking DB URL:", process.env.DATABASE_URL ? "Loaded ✅" : "Missing ❌");
console.log("🔍 DB URL value:", process.env.DATABASE_URL?.substring(0, 30) + "...");

import { auth } from './auth.config';
import http from 'http';
import url from 'url';

// Validate environment
const PORT = parseInt(process.env.AUTH_PORT || '3001', 10);
const HOST = process.env.AUTH_HOST || 'localhost';

/**
 * Request handler for auth routes
 */
async function requestHandler(req: http.IncomingMessage, res: http.ServerResponse) {
  // Enable CORS with explicit origins (required for credentials)
  const allowedOrigins = ['http://localhost:3000', 'http://localhost:8000'];
  const origin = req.headers.origin || '';

  if (allowedOrigins.includes(origin)) {
    res.setHeader('Access-Control-Allow-Origin', origin);
  }

  res.setHeader('Access-Control-Allow-Credentials', 'true');
  res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization');

  if (req.method === 'OPTIONS') {
    res.writeHead(200);
    res.end();
    return;
  }

  const parsedUrl = url.parse(req.url || '', true);
  const pathname = parsedUrl.pathname || '';

  try {
    // Health check endpoint
    if (pathname === '/api/auth/health' && req.method === 'GET') {
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(
        JSON.stringify({
          status: 'healthy',
          service: 'auth',
          version: '1.0.0',
          timestamp: new Date().toISOString(),
        })
      );
      return;
    }

    // Root endpoint
    if (pathname === '/api/auth' && req.method === 'GET') {
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(
        JSON.stringify({
          message: 'Physical AI Textbook Auth Service',
          docs: '/api/auth/docs',
        })
      );
      return;
    }

    // Route to better-auth handler
    // Note: better-auth returns a Web API Response
    if (pathname.startsWith('/api/auth')) {
      // Convert Node.js IncomingMessage to Web API Request
      const request = new Request(`http://${req.headers.host}${req.url}`, {
        method: req.method,
        headers: req.headers as any,
      });

      const response = await auth.handler(request);
      if (response) {
        // Convert Web API Response to Node.js response
        const headers: any = {};
        response.headers.forEach((value, key) => {
          headers[key] = value;
        });
        res.writeHead(response.status, headers);
        res.end(await response.text());
        return;
      }
    }

    // 404
    res.writeHead(404, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ error: 'Not Found' }));
  } catch (error) {
    console.error('Error handling request:', error);
    res.writeHead(500, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ error: 'Internal Server Error' }));
  }
}

/**
 * Create and start HTTP server
 */
const server = http.createServer(requestHandler);

server.listen(PORT, HOST, () => {
  console.log(`🔐 Auth server running on http://${HOST}:${PORT}`);
  console.log(`📚 API endpoints at http://${HOST}:${PORT}/api/auth`);
  console.log(`💾 Database: Neon Postgres`);
  console.log(`📝 Endpoints: /api/auth/signup, /api/auth/login, /api/auth/signout, /api/auth/session`);
});

server.on('error', (error) => {
  console.error('Server error:', error);
  process.exit(1);
});

process.on('SIGTERM', () => {
  console.log('SIGTERM received, shutting down gracefully...');
  server.close(() => {
    console.log('Server closed');
    process.exit(0);
  });
});

export default server;
