/**
 * Database Connection
 *
 * Sets up the database connection using Neon Postgres
 */

import { config } from 'dotenv';
import { resolve, dirname } from 'path';
import { fileURLToPath } from 'url';
import { neon } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';
import * as schema from './simple-schema.js';

// ES module equivalent of __dirname
const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Load .env from repository root
config({ path: resolve(__dirname, '../../../.env'), override: true });

// Validate DATABASE_URL
if (!process.env.DATABASE_URL) {
  throw new Error('❌ DATABASE_URL is missing. Please check your .env file.');
}

// Initialize Neon Postgres connection
const sql = neon(process.env.DATABASE_URL);

// Initialize Drizzle ORM
export const db = drizzle(sql as any, { schema });

export default db;
