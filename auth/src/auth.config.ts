/**
 * Better-Auth Configuration
 *
 * Configures the authentication service with:
 * - Neon Postgres database connection
 * - Extended user schema with hardware_bg field
 * - Email/password and username credential providers
 * - Session management
 */

import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { postgres } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';
import * as schema from './db/schema';

// Validate environment variables
if (!process.env.DATABASE_URL) {
  throw new Error('DATABASE_URL environment variable is not set');
}

/**
 * Initialize Neon Postgres connection
 * Uses serverless driver for edge/lambda compatibility
 */
const client = postgres(process.env.DATABASE_URL);

/**
 * Initialize Drizzle ORM with Neon client
 */
const db = drizzle(client, { schema });

/**
 * Configure Better-Auth with Neon adapter
 */
export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: 'pg', // Postgres provider
    usernameTable: 'user', // Use the user table for usernames
  }),

  /**
   * Email/Password authentication provider
   * Allows users to sign up and log in with email and password
   */
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // For demo/hackathon, disable verification
  },

  /**
   * Username/Password authentication provider
   * Alternative to email-based authentication
   */
  username: {
    enabled: true,
  },

  /**
   * Session configuration
   */
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // Update every 24 hours
    absoluteExpiresIn: 60 * 60 * 24 * 30, // Absolute expiry: 30 days
  },

  /**
   * JWT configuration for token-based auth
   */
  jwt: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    secret: process.env.JWT_SECRET || 'your-secret-key-change-in-production',
  },

  /**
   * CORS configuration
   */
  cors: {
    origin: [
      'http://localhost:3000', // Docusaurus dev server
      'http://localhost:8000', // FastAPI server
      process.env.FRONTEND_URL || '',
    ].filter(Boolean),
    credentials: true,
    allowedHeaders: ['content-type', 'authorization'],
  },

  /**
   * Plugins and middleware
   */
  plugins: [
    // Plugin hooks can be added here for custom behavior
  ],

  /**
   * Custom user fields
   * Extends the default user model with hardware_bg and software_bg
   * These are handled via the schema extension in db/schema.ts
   */
  advancedOptions: {
    allowSignUpWithUnverifiedEmail: true, // Allow signup without email verification
  },
});

export default auth;
