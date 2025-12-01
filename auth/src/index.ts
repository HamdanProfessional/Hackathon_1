/**
 * Authentication Server
 *
 * Runs the better-auth service on port 3001
 * Provides endpoints for signup, login, and session management
 */

import 'dotenv/config';
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
  // Enable CORS
  res.setHeader('Access-Control-Allow-Origin', '*');
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
    // Note: better-auth has built-in HTTP handler support
    if (pathname.startsWith('/api/auth')) {
      const betterAuthHandler = await auth.handler(req, res);
      if (betterAuthHandler) {
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
