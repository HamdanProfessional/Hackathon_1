import type { VercelRequest, VercelResponse } from '@vercel/node';
import { neon } from '@neondatabase/serverless';

/**
 * Database Initialization Endpoint
 *
 * This endpoint creates the users table if it doesn't exist
 * Call this once after deployment to initialize the database
 *
 * Usage: GET /api/init-db
 */

export default async function handler(req: VercelRequest, res: VercelResponse) {
  // Only allow GET requests
  if (req.method !== 'GET') {
    res.status(405).json({ error: 'Method not allowed' });
    return;
  }

  try {
    // Check if DATABASE_URL is set
    if (!process.env.DATABASE_URL) {
      res.status(500).json({ error: 'DATABASE_URL is not configured' });
      return;
    }

    const sql = neon(process.env.DATABASE_URL);

    // Create users table
    const createTableSQL = `
      CREATE TABLE IF NOT EXISTS users (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        email TEXT NOT NULL UNIQUE,
        password_hash TEXT NOT NULL,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL,
        updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL
      );
    `;

    console.log('Creating users table...');
    await sql(createTableSQL);

    // Create index on email
    const createIndexSQL = `
      CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);
    `;

    console.log('Creating email index...');
    await sql(createIndexSQL);

    res.status(200).json({
      success: true,
      message: 'Database initialized successfully',
      tables: ['users'],
    });
  } catch (error) {
    console.error('Database initialization error:', error);
    res.status(500).json({
      error: 'Database initialization failed',
      message: error instanceof Error ? error.message : 'Unknown error',
    });
  }
}
