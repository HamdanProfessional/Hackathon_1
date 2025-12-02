// 1. LOAD ENV VARS FIRST
import { config } from 'dotenv';
import { resolve } from 'path';

// Load .env from parent directory (repository root)
// override: true forces .env to override system environment variables
config({ path: resolve(__dirname, '../../.env'), override: true });

/**
 * Better-Auth Configuration
 *
 * Path: auth/src/auth.config.ts
 */ 

import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { neon } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';
import * as schema from './db/schema';

// 2. VALIDATE ENV VARS
// Ensure DATABASE_URL is set in your .env file inside the /auth folder
if (!process.env.DATABASE_URL) {
  throw new Error('❌ DATABASE_URL is missing. Please check your .env file.');
}

/**
 * Initialize Neon Postgres connection
 */
const sql = neon(process.env.DATABASE_URL);

/**
 * Initialize Drizzle ORM
 * We cast 'sql' to 'any' to fix the strict type mismatch between 
 * @neondatabase/serverless and drizzle-orm versions.
 */
const db = drizzle(sql as any, { schema });

/**
 * Configure Better-Auth
 */
export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: 'pg',
    // Note: We rely on the default table names (user, session, etc.)
    // defined in your schema.ts
  }),

  /**
   * User Schema Extensions
   * Allows 'hardware_bg' and 'software_bg' to be passed during signup
   */
  user: {
    additionalFields: {
      hardware_bg: {
        type: "string",
        required: false,
        defaultValue: "Laptop",
        input: true
      },
      software_bg: {
        type: "string",
        required: false,
        input: true
      },
      skill_level: {
        type: "string",
        required: false,
        defaultValue: "Beginner",
        input: true
      }
    }
  },

  /**
   * Authentication Providers
   */
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, 
  },

  username: {
    enabled: true,
  },

  /**
   * Session & Security
   */
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, 
  },

  cors: {
    origin: [
      'http://localhost:3000', 
      'http://localhost:8000', 
      process.env.FRONTEND_URL || '',
    ].filter(Boolean),
    credentials: true,
    allowedHeaders: ['content-type', 'authorization'],
  },

  advancedOptions: {
    allowSignUpWithUnverifiedEmail: true,
  },
});

export default auth;